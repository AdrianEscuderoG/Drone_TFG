/**
 * @file main.c
 * @brief ESP32-S3 ESCLAVO — Captura OV3660 y entrega frames por SPI
 *
 * NOTA: spi_slave_task vuelve al modelo secuencial (spi_slave_transmit
 * bloqueante), tras confirmar que el modelo de doble buffer con
 * modificacion de buffer post-encolado NO es fiable en el driver SPI
 * esclavo del ESP32 (el contenido puede quedar fijado en el momento
 * de encolar, no en el momento real de la transferencia). El modelo
 * secuencial, con delay ampliado a 5ms y prioridades camera=6/spi=8,
 * es el que demostro funcionar en las pruebas de hardware.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * CONFIGURACIÓN
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5
#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

#define CAM_SYNC_GPIO   38

#define SPI_HOST_ID     SPI2_HOST
#define SPI_CLK_PIN     41
#define SPI_MOSI_PIN    42
#define SPI_MISO_PIN    40
#define SPI_CS_PIN      39

#define CMD_QUERY       0xAA
#define CMD_READ_CHUNK  0xA5
#define CMD_ACK_DONE    0xA6
#define RESP_QUERY      0xBB
#define RESP_CHUNK      0xA5
#define RESP_ACK        0xA6

#define CHUNK_SIZE      4096
#define TRANS_HEADER    4
#define TRANS_SIZE      (TRANS_HEADER + CHUNK_SIZE)
#define FRAME_BUF_MAX   (60 * 1024)   // subido a 60KB para VGA, antes 32KB para QVGA

/* ═══════════════════════════════════════════════════════════════════════════
 * ESTADO GLOBAL
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *TAG = "slave_cam";

static uint8_t          *g_frame_buf          = NULL;
static size_t             g_frame_size        = 0;
static int64_t            g_frame_timestamp_us = 0;
static volatile bool      g_frame_ready       = false;
static SemaphoreHandle_t  g_frame_mutex       = NULL;

static SemaphoreHandle_t  g_trigger_sem       = NULL;

static uint8_t *g_spi_tx = NULL;
static uint8_t *g_spi_rx = NULL;

/* ═══════════════════════════════════════════════════════════════════════════
 * INICIALIZACIÓN DE CÁMARA
 * ═══════════════════════════════════════════════════════════════════════════ */

static esp_err_t camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7       = CAM_PIN_D7,
        .pin_d6       = CAM_PIN_D6,
        .pin_d5       = CAM_PIN_D5,
        .pin_d4       = CAM_PIN_D4,
        .pin_d3       = CAM_PIN_D3,
        .pin_d2       = CAM_PIN_D2,
        .pin_d1       = CAM_PIN_D1,
        .pin_d0       = CAM_PIN_D0,
        .pin_vsync    = CAM_PIN_VSYNC,
        .pin_href     = CAM_PIN_HREF,
        .pin_pclk     = CAM_PIN_PCLK,
        .xclk_freq_hz = 10000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_VGA, //antes .frame_size   = FRAMESIZE_QVGA,
        .jpeg_quality = 15,
        .fb_count     = 1,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init fallo: 0x%x", err);
    }
    return err;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ISR: trigger de sincronia estereo
 * ═══════════════════════════════════════════════════════════════════════════ */

static void IRAM_ATTR sync_isr_handler(void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(g_trigger_sem, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TAREA: captura de cámara — reactiva al trigger
 * ═══════════════════════════════════════════════════════════════════════════ */

static void camera_task(void *arg)
{
    ESP_LOGI(TAG, "[cam] esperando triggers en GPIO%d...", CAM_SYNC_GPIO);

    while (true) {
        xSemaphoreTake(g_trigger_sem, portMAX_DELAY);

        xSemaphoreTake(g_frame_mutex, portMAX_DELAY);
        bool previous_pending = g_frame_ready;
        xSemaphoreGive(g_frame_mutex);

        if (previous_pending) {
            ESP_LOGW(TAG, "[cam] trigger ignorado: frame anterior sin confirmar");
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "[cam] Captura fallida");
            continue;
        }

        if (fb->format == PIXFORMAT_JPEG && fb->len > 0
            && fb->len <= FRAME_BUF_MAX) {
            xSemaphoreTake(g_frame_mutex, portMAX_DELAY);
            memcpy(g_frame_buf, fb->buf, fb->len);
            g_frame_size         = fb->len;
            g_frame_timestamp_us = esp_timer_get_time();
            g_frame_ready        = true;
            xSemaphoreGive(g_frame_mutex);
        }

        esp_camera_fb_return(fb);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TAREA: SPI esclavo — modelo secuencial (probado y funcional)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void spi_slave_task(void *arg)
{
    spi_slave_transaction_t trans = {
        .length    = TRANS_SIZE * 8,
        .tx_buffer = g_spi_tx,
        .rx_buffer = g_spi_rx,
    };

    ESP_LOGI(TAG, "[spi] Esperando master...");

    while (true) {
        // Fase A: recibir CMD, enviar ceros
        memset(g_spi_tx, 0x00, TRANS_SIZE);
        if (spi_slave_transmit(SPI_HOST_ID, &trans,
                               portMAX_DELAY) != ESP_OK) {
            continue;
        }

        uint8_t  cmd       = g_spi_rx[0];
        uint16_t chunk_idx = ((uint16_t)g_spi_rx[1] << 8) | g_spi_rx[2];

        memset(g_spi_tx, 0x00, TRANS_SIZE);

        if (cmd == CMD_QUERY) {
            xSemaphoreTake(g_frame_mutex, pdMS_TO_TICKS(10));
            bool    ready = g_frame_ready;
            size_t  size  = g_frame_size;
            int64_t ts    = g_frame_timestamp_us;
            xSemaphoreGive(g_frame_mutex);

            g_spi_tx[0] = RESP_QUERY;
            g_spi_tx[1] = ready ? 1 : 0;
            g_spi_tx[2] = (size >> 8) & 0xFF;
            g_spi_tx[3] = size & 0xFF;

            for (int i = 0; i < 8; i++) {
                g_spi_tx[TRANS_HEADER + i] = (uint8_t)((ts >> (56 - i * 8)) & 0xFF);
            }

        } else if (cmd == CMD_READ_CHUNK) {
            g_spi_tx[0] = RESP_CHUNK;
            g_spi_tx[1] = g_spi_rx[1];
            g_spi_tx[2] = g_spi_rx[2];
            g_spi_tx[3] = 0;

            size_t offset = (size_t)chunk_idx * CHUNK_SIZE;

            xSemaphoreTake(g_frame_mutex, portMAX_DELAY);
            if (g_frame_ready && offset < g_frame_size) {
                size_t bytes = g_frame_size - offset;
                if (bytes > CHUNK_SIZE) bytes = CHUNK_SIZE;
                memcpy(&g_spi_tx[TRANS_HEADER],
                       &g_frame_buf[offset], bytes);
            }
            xSemaphoreGive(g_frame_mutex);

        } else if (cmd == CMD_ACK_DONE) {
            xSemaphoreTake(g_frame_mutex, portMAX_DELAY);
            g_frame_ready = false;
            xSemaphoreGive(g_frame_mutex);
            g_spi_tx[0] = RESP_ACK;
            ESP_LOGI(TAG, "[spi] Frame confirmado, size=%zu B",
                     g_frame_size);
        } else {
            g_spi_tx[0] = 0xFF;
            ESP_LOGW(TAG, "[spi] CMD desconocido: 0x%02X", cmd);
        }

        // Fase B: enviar respuesta
        spi_slave_transmit(SPI_HOST_ID, &trans, portMAX_DELAY);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUNTO DE ENTRADA
 * ═══════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== Slave CAM iniciando ===");

    g_frame_buf = heap_caps_malloc(FRAME_BUF_MAX, MALLOC_CAP_SPIRAM);
    if (!g_frame_buf) {
        ESP_LOGE(TAG, "PSRAM no disponible");
        return;
    }

    g_spi_tx = heap_caps_malloc(TRANS_SIZE,
                                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    g_spi_rx = heap_caps_malloc(TRANS_SIZE,
                                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!g_spi_tx || !g_spi_rx) {
        ESP_LOGE(TAG, "Sin RAM interna para DMA");
        return;
    }

    g_frame_mutex = xSemaphoreCreateMutex();
    g_trigger_sem = xSemaphoreCreateBinary();

    gpio_config_t sync_cfg = {
        .pin_bit_mask = (1ULL << CAM_SYNC_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&sync_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CAM_SYNC_GPIO, sync_isr_handler, NULL);

    ESP_ERROR_CHECK(camera_init());
    ESP_LOGI(TAG, "Camara lista (OV3660, JPEG QVGA)");

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = SPI_MOSI_PIN,
        .miso_io_num     = SPI_MISO_PIN,
        .sclk_io_num     = SPI_CLK_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = TRANS_SIZE,
    };
    spi_slave_interface_config_t slave_cfg = {
        .mode         = 0,
        .spics_io_num = SPI_CS_PIN,
        .queue_size   = 1,
        .flags        = 0,
    };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST_ID, &bus_cfg,
                                         &slave_cfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI esclavo listo");

    xTaskCreate(camera_task,    "camera",    4096, NULL, 6, NULL);
    xTaskCreate(spi_slave_task, "spi_slave", 8192, NULL, 8, NULL);

    ESP_LOGI(TAG, "Tareas creadas. Listo, esperando triggers del master.");
}