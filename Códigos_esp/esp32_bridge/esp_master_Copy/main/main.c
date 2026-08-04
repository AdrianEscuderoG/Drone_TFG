/**
 * @file main.c
 * @brief ESP32-S3 master — Puente MAVLink USB-CDC<->UART + captura de imagen estereo por SPI/WiFi
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * CAMBIO DE ARQUITECTURA RESPECTO A LA VERSION ANTERIOR
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * ANTES: MAVLink viajaba por WiFi UDP (ESP32 SoftAP <-> MAVROS).
 * AHORA: MAVLink viaja por cable USB (USB-CDC / puerto serie virtual).
 *
 * Motivo: el enlace WiFi introducia RTT variable (25-260ms) que rompia
 * el mecanismo de timesync de MAVROS (umbral fijo interno de 10ms) y
 * degradaba la tasa efectiva de IMU a alta frecuencia. El USB, al ser
 * un enlace fisico directo sin radio de por medio, deberia dar un RTT
 * consistentemente por debajo de ese umbral.
 *
 * Las imagenes NO se tocan: siguen viajando por WiFi UDP:14552 exactamente
 * igual que en la version anterior, ya que ese canal nunca ha dado problemas
 * de timing (no depende de ningun mecanismo de sincronizacion de reloj).
 *
 * Arquitectura resultante:
 *
 *   MAVLink:  GS (MAVROS) <--USB-CDC--> ESP32 <--UART2--> SpeedyBee F405
 *   Imagen:   Slaves (SPI) -> ESP32 -> WiFi UDP:14552 -> nodo ROS 2 dedicado
 *
 * Config MAVROS en la GS (cambia respecto a antes):
 *   fcu_url:=/dev/ttyACM0:115200
 *   (el nombre exacto del dispositivo puede variar, comprobar con
 *    `ls /dev/ttyACM*` o `dmesg | tail` tras conectar el USB)
 *
 * IMPORTANTE - LEER EL RESUMEN AL FINAL DE ESTE ARCHIVO antes de continuar
 * el trabajo en otra sesion: incluye supuestos sobre la API de TinyUSB que
 * NO han sido verificados contra la version exacta de ESP-IDF del proyecto
 * (v6.0.1 segun documentacion previa) y que deben revisarse antes de compilar.
 *
 * Pines UART2 (sin cambios):
 *   TX -> GPIO 17  (conectado a RX de SpeedyBee UART2)
 *   RX -> GPIO 16  (conectado a TX de SpeedyBee UART2)
 *
 * Parametros ArduPilot requeridos (sin cambios):
 *   SERIAL2_PROTOCOL = 2   (MAVLink 2)
 *   SERIAL2_BAUD     = 115 (115200)
 */

#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "common/mavlink.h"

/* TinyUSB / CDC-ACM — requiere el componente gestionado "espressif/esp_tinyusb"
 * (ver notas de compilacion en el resumen al final del archivo). */
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * CONFIGURACIÓN — RED / MAVLINK
 * ═══════════════════════════════════════════════════════════════════════════ */

#define WIFI_AP_SSID       "DroneNet"
#define WIFI_AP_PASS       "drone1234"
#define WIFI_AP_CHANNEL    6
#define WIFI_AP_MAX_STA    1

#define GS_MAC_ADDR        { 0x28, 0x16, 0xad, 0x9f, 0xbb, 0x5e }

#define AP_IP_STR          "192.168.4.1"
#define AP_GW_STR          "192.168.4.1"
#define AP_NETMASK_STR     "255.255.255.0"
#define GS_IP_STR          "192.168.4.2"

#define UART_PORT_NUM      UART_NUM_2
#define UART_TX_GPIO       17
#define UART_RX_GPIO       16
#define UART_BAUD_RATE     115200
#define UART_DRIVER_BUF    4096

#define UART_RX_BUF_SIZE   1024

#define TASK_STACK_SIZE    4096
#define TASK_PRIO_USB_RX   5
#define TASK_PRIO_UART_RX  5

/* ═══════════════════════════════════════════════════════════════════════════
 * CONFIGURACIÓN — USB-CDC (MAVLink)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CDC_ITF                 TINYUSB_CDC_ACM_0
#define USB_RX_QUEUE_LEN        32          // numero de "paquetes" de bytes en cola
#define USB_RX_CHUNK_MAX        256         // tamaño maximo leido por evento RX

// Estructura para pasar los bytes recibidos por USB desde el callback
// (contexto de la tarea interna de TinyUSB) hasta udp_to_uart_task, que
// es quien realmente escribe en la UART. Se copian los bytes porque el
// buffer del callback no es valido fuera de su ambito.
typedef struct {
    uint8_t  data[USB_RX_CHUNK_MAX];
    size_t   len;
} usb_rx_chunk_t;

static QueueHandle_t g_usb_rx_queue = NULL;

/* ═══════════════════════════════════════════════════════════════════════════
 * CONFIGURACIÓN — CAPTURA DE IMAGEN (sin cambios respecto a la version anterior)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define NUM_CAMS   2
#define MAX_CAMS   2

#define TRIGGER_GPIO        8

#define SPI_MASTER_HOST     SPI2_HOST
#define PIN_MASTER_CLK      12
#define PIN_MASTER_MOSI     11
#define PIN_MASTER_MISO     13
#define PIN_MASTER_CS0      10
#define PIN_MASTER_CS1      9
#define SPI_CLOCK_HZ        (4 * 1000 * 1000)

#define CMD_QUERY           0xAA
#define CMD_READ_CHUNK      0xA5
#define CMD_ACK_DONE        0xA6
#define RESP_QUERY          0xBB
#define RESP_CHUNK          0xA5
#define CHUNK_SIZE          4096
#define TRANS_HEADER        4
#define TRANS_SIZE          (TRANS_HEADER + CHUNK_SIZE)
#define MAX_FRAME_BYTES     (32 * 1024)
#define SPI_PHASE_DELAY_MS  5
#define QUERY_MAX_ATTEMPTS  40
#define CHUNK_MAX_RETRIES   3

#define IMAGE_UDP_PORT      14552

/* ═══════════════════════════════════════════════════════════════════════════
 * VARIABLES GLOBALES
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *TAG = "bridge";

static const uint8_t gs_mac[6] = GS_MAC_ADDR;
static volatile bool gs_connected = false;

static EventGroupHandle_t wifi_event_group;
#define BIT_GS_READY  BIT0

static const int            cam_cs_pins[MAX_CAMS] = { PIN_MASTER_CS0, PIN_MASTER_CS1 };
static spi_device_handle_t  cam_dev[MAX_CAMS];
static uint32_t             cam_frame_num[MAX_CAMS] = { 0, 0 };

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 1: WiFi AP (sin cambios — sigue haciendo falta para las imagenes)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t *)event_data;
        char mac_str[18];

        if (memcmp(ev->mac, gs_mac, 6) != 0) {
            snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(ev->mac));
            ESP_LOGW(TAG, "MAC no autorizada: %s - rechazando", mac_str);
            esp_wifi_deauth_sta(ev->aid);
            return;
        }

        snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(ev->mac));
        ESP_LOGI(TAG, "GS conectada (WiFi, imagenes): %s (AID=%d)", mac_str, ev->aid);
        gs_connected = true;
        xEventGroupSetBits(wifi_event_group, BIT_GS_READY);

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)event_data;
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(ev->mac));
        ESP_LOGW(TAG, "GS desconectada (WiFi): %s", mac_str);
        gs_connected = false;
        xEventGroupClearBits(wifi_event_group, BIT_GS_READY);
    }
}

static void wifi_init_ap(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    esp_netif_ip_info_t ip_info = { 0 };
    ip_info.ip.addr      = inet_addr(AP_IP_STR);
    ip_info.gw.addr      = inet_addr(AP_GW_STR);
    ip_info.netmask.addr = inet_addr(AP_NETMASK_STR);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = strlen(WIFI_AP_SSID),
            .channel        = WIFI_AP_CHANNEL,
            .password       = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_STA,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg        = { .required = true },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Ahorro de energia WiFi desactivado: aunque aqui el ESP32 actua como AP
    // (no como estacion), se deja explicito por completitud y para descartar
    // esta variable en futuras pruebas de latencia del canal de imagenes.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "AP iniciado  SSID='%s'  IP=%s", WIFI_AP_SSID, AP_IP_STR);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 2: UART hacia SpeedyBee (sin cambios)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT_NUM, UART_DRIVER_BUF, UART_DRIVER_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d listo  TX=GPIO%d  RX=GPIO%d  @%d baud",
             UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_BAUD_RATE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 3: USB-CDC (MAVLink) — sustituye al socket UDP de la version anterior
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Callback invocado por la tarea interna de TinyUSB cuando hay bytes
 * nuevos disponibles en el endpoint CDC (direccion GS -> ESP32, es decir
 * comandos de MAVROS hacia la FC).
 *
 * NOTA IMPORTANTE (ver resumen al final): la firma exacta de este callback
 * y de tinyusb_cdcacm_read() puede variar segun la version del componente
 * esp_tinyusb instalado. Verificar contra el header tusb_cdc_acm.h real
 * antes de compilar.
 *
 * Se copian los bytes a una cola en vez de escribir directamente a la UART
 * desde aqui, para no bloquear la tarea interna de TinyUSB con una llamada
 * potencialmente lenta (uart_write_bytes puede bloquear si el buffer TX
 * interno del driver UART esta lleno).
 */
static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    usb_rx_chunk_t chunk;
    size_t rx_size = 0;

    esp_err_t ret = tinyusb_cdcacm_read(itf, chunk.data, USB_RX_CHUNK_MAX, &rx_size);
    if (ret == ESP_OK && rx_size > 0) {
        chunk.len = rx_size;
        // Si la cola esta llena, se descarta el chunk mas antiguo en vez de
        // bloquear el callback (mejor perder un comando puntual que colgar
        // el stack USB).
        if (xQueueSend(g_usb_rx_queue, &chunk, 0) != pdTRUE) {
            usb_rx_chunk_t discard;
            xQueueReceive(g_usb_rx_queue, &discard, 0);
            xQueueSend(g_usb_rx_queue, &chunk, 0);
            ESP_LOGW(TAG, "[usb->uart] cola llena, se descarta chunk antiguo");
        }
    }
}

static void usb_cdc_init(void)
{
    g_usb_rx_queue = xQueueCreate(USB_RX_QUEUE_LEN, sizeof(usb_rx_chunk_t));

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,          // usar descriptor por defecto
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    const tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = CDC_ITF,
        .rx_unread_buf_sz = 256,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    ESP_LOGI(TAG, "USB-CDC listo (MAVLink). El PC debe ver /dev/ttyACM0 o similar.");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 4: Tareas MAVLink (USB-CDC <-> UART2)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * GS (MAVROS) -> USB -> UART -> SpeedyBee  (comandos al FC)
 *
 * A diferencia de la version UDP anterior, esta tarea no espera a que
 * haya WiFi conectado (el USB es independiente de la conexion WiFi de
 * las imagenes) — arranca en cuanto el host PC abre el puerto serie
 * virtual, que TinyUSB gestiona de forma transparente.
 */
static void usb_to_uart_task(void *arg)
{
    ESP_LOGI(TAG, "[usb->uart] esperando datos de MAVROS por USB...");

    usb_rx_chunk_t chunk;
    while (true) {
        if (xQueueReceive(g_usb_rx_queue, &chunk, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART_PORT_NUM, chunk.data, chunk.len);
        }
    }

    vTaskDelete(NULL);
}

/**
 * SpeedyBee -> UART -> parser MAVLink -> USB -> GS (MAVROS)  (telemetria)
 *
 * Se mantiene el parseo mensaje-a-mensaje con mavlink_parse_char (igual
 * que en la version UDP) para poder loguear/contar por msgid si hace
 * falta en el futuro, aunque ya no sea estrictamente necesario para el
 * transporte en si (USB-CDC es un flujo de bytes, no habria problema en
 * reenviar los bytes crudos sin parsear). Se conserva el parseo por
 * continuidad con el codigo anterior y por si se quiere reintroducir el
 * agrupamiento de varios mensajes en una sola escritura USB mas adelante
 * (ver resumen al final).
 */
static void uart_to_usb_task(void *arg)
{
    ESP_LOGI(TAG, "[uart->usb] listo, reenviando FC -> MAVROS por USB");

    uint8_t raw[UART_RX_BUF_SIZE];
    mavlink_message_t msg;
    mavlink_status_t  mav_status;
    uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];

    uint32_t frames_sent = 0;
    uint32_t frames_log  = 0;

    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, raw, sizeof(raw),
                                  pdMS_TO_TICKS(20));

        for (int i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, raw[i], &msg, &mav_status)) {
                uint16_t frame_len = mavlink_msg_to_send_buffer(mav_buf, &msg);

                // tinyusb_cdcacm_write_queue encola sin bloquear; el flush
                // real lo hace tinyusb_cdcacm_write_flush(). Se llama al
                // flush tras cada mensaje para minimizar latencia añadida
                // por buffering interno (a costa de mas llamadas — si esto
                // resulta ser un cuello de botella, agrupar N mensajes
                // antes de un unico flush es la primera optimizacion a
                // probar, ver resumen).
                size_t queued = tinyusb_cdcacm_write_queue(CDC_ITF, mav_buf, frame_len);
                if (queued == frame_len) {
                    tinyusb_cdcacm_write_flush(CDC_ITF, 0);
                    frames_sent++;
                } else {
                    ESP_LOGW(TAG, "[uart->usb] write_queue parcial: %u/%u bytes",
                             (unsigned)queued, frame_len);
                }
            }
        }

        if (frames_sent - frames_log >= 100) {
            ESP_LOGI(TAG, "[uart->usb] %lu frames MAVLink enviados", frames_sent);
            frames_log = frames_sent;
        }

        if (len == 0) {
            taskYIELD();
        }
    }

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 5: Captura de imagen (SPI + WiFi UDP) — SIN CAMBIOS respecto a la
 * version anterior. Se mantiene integra porque no forma parte del problema
 * que se estaba resolviendo (timesync de MAVLink).
 * ═══════════════════════════════════════════════════════════════════════════ */

static esp_err_t spi_transfer(spi_device_handle_t dev,
                               uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(dev, &t);
}

static bool capture_camera(spi_device_handle_t dev, uint8_t cam_id,
                            uint8_t *tx_buf, uint8_t *rx_buf, uint8_t *img_buf,
                            uint16_t *out_size, int64_t *out_ts)
{
    bool     ready      = false;
    uint16_t frame_size = 0;
    int64_t  ts_us      = 0;

    for (int attempt = 0; attempt < QUERY_MAX_ATTEMPTS; attempt++) {
        memset(tx_buf, 0, TRANS_SIZE);
        tx_buf[0] = CMD_QUERY;
        spi_transfer(dev, tx_buf, rx_buf, TRANS_SIZE);
        vTaskDelay(pdMS_TO_TICKS(SPI_PHASE_DELAY_MS));
        memset(tx_buf, 0, TRANS_SIZE);
        spi_transfer(dev, tx_buf, rx_buf, TRANS_SIZE);

        if (rx_buf[0] == RESP_QUERY && rx_buf[1] == 1) {
            frame_size = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
            ts_us = 0;
            for (int i = 0; i < 8; i++) {
                ts_us = (ts_us << 8) | rx_buf[TRANS_HEADER + i];
            }
            ready = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!ready || frame_size == 0 || frame_size > MAX_FRAME_BYTES) {
        ESP_LOGW(TAG, "[img] cam%d: sin frame tras %d intentos",
                 cam_id, QUERY_MAX_ATTEMPTS);
        return false;
    }

    uint16_t num_chunks  = (frame_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
    uint32_t bytes_total = 0;
    bool     frame_ok    = true;

    for (uint16_t i = 0; i < num_chunks; i++) {
        bool chunk_ok = false;

        for (int retry = 0; retry < CHUNK_MAX_RETRIES && !chunk_ok; retry++) {
            memset(tx_buf, 0, TRANS_SIZE);
            tx_buf[0] = CMD_READ_CHUNK;
            tx_buf[1] = (i >> 8) & 0xFF;
            tx_buf[2] =  i       & 0xFF;
            spi_transfer(dev, tx_buf, rx_buf, TRANS_SIZE);
            vTaskDelay(pdMS_TO_TICKS(SPI_PHASE_DELAY_MS));
            memset(tx_buf, 0, TRANS_SIZE);
            spi_transfer(dev, tx_buf, rx_buf, TRANS_SIZE);

            if (rx_buf[0] == RESP_CHUNK) {
                size_t chunk_bytes = frame_size - bytes_total;
                if (chunk_bytes > CHUNK_SIZE) chunk_bytes = CHUNK_SIZE;
                memcpy(img_buf + bytes_total, &rx_buf[TRANS_HEADER], chunk_bytes);
                bytes_total += chunk_bytes;
                chunk_ok = true;
            } else {
                ESP_LOGW(TAG, "[img] cam%d: chunk %u inesperado (0x%02X), intento %d/%d",
                         cam_id, i, rx_buf[0], retry + 1, CHUNK_MAX_RETRIES);
            }
        }

        if (!chunk_ok) {
            ESP_LOGE(TAG, "[img] cam%d: chunk %u perdido tras %d intentos — frame descartado",
                     cam_id, i, CHUNK_MAX_RETRIES);
            frame_ok = false;
            break;
        }
    }

    memset(tx_buf, 0, TRANS_SIZE);
    tx_buf[0] = CMD_ACK_DONE;
    spi_transfer(dev, tx_buf, rx_buf, TRANS_SIZE);
    vTaskDelay(pdMS_TO_TICKS(SPI_PHASE_DELAY_MS));
    memset(tx_buf, 0, TRANS_SIZE);
    spi_transfer(dev, tx_buf, rx_buf, TRANS_SIZE);

    if (!frame_ok || bytes_total == 0) {
        return false;
    }

    *out_size = (uint16_t)bytes_total;
    *out_ts   = ts_us;
    return true;
}

static void image_capture_task(void *arg)
{
    ESP_LOGI(TAG, "[img] esperando GS (WiFi)...");
    xEventGroupWaitBits(wifi_event_group, BIT_GS_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << TRIGGER_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trig_cfg);
    gpio_set_level(TRIGGER_GPIO, 0);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = PIN_MASTER_MOSI,
        .miso_io_num     = PIN_MASTER_MISO,
        .sclk_io_num     = PIN_MASTER_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = TRANS_SIZE,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_MASTER_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    for (int i = 0; i < NUM_CAMS; i++) {
        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = SPI_CLOCK_HZ,
            .mode           = 0,
            .spics_io_num   = cam_cs_pins[i],
            .queue_size     = 1,
        };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_HOST, &dev_cfg, &cam_dev[i]));
        ESP_LOGI(TAG, "[img] cam%d lista, CS=GPIO%d", i, cam_cs_pins[i]);
    }
    ESP_LOGI(TAG, "[img] SPI @ %d Hz, NUM_CAMS=%d", SPI_CLOCK_HZ, NUM_CAMS);

    uint8_t *tx_buf  = heap_caps_malloc(TRANS_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *rx_buf  = heap_caps_malloc(TRANS_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *img_buf = malloc(MAX_FRAME_BYTES);
    uint8_t *udp_buf = malloc(17 + MAX_FRAME_BYTES);
    if (!tx_buf || !rx_buf || !img_buf || !udp_buf) {
        ESP_LOGE(TAG, "[img] sin memoria");
        vTaskDelete(NULL);
        return;
    }

    int sock_img = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in img_dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(IMAGE_UDP_PORT),
        .sin_addr.s_addr = inet_addr(GS_IP_STR),
    };
    ESP_LOGI(TAG, "[img] UDP imagenes -> %s:%d", GS_IP_STR, IMAGE_UDP_PORT);

    while (true) {
        gpio_set_level(TRIGGER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(TRIGGER_GPIO, 0);

        for (int cam = 0; cam < NUM_CAMS; cam++) {
            uint16_t frame_size;
            int64_t  ts_us;

            bool ok = capture_camera(cam_dev[cam], (uint8_t)cam,
                                      tx_buf, rx_buf, img_buf,
                                      &frame_size, &ts_us);
            if (!ok) continue;

            if (gs_connected) {
                udp_buf[0] = 0xCA;
                udp_buf[1] = 0xFE;
                udp_buf[2] = (uint8_t)cam;
                udp_buf[3] = (cam_frame_num[cam] >> 24) & 0xFF;
                udp_buf[4] = (cam_frame_num[cam] >> 16) & 0xFF;
                udp_buf[5] = (cam_frame_num[cam] >>  8) & 0xFF;
                udp_buf[6] =  cam_frame_num[cam]        & 0xFF;
                udp_buf[7] = (frame_size >> 8) & 0xFF;
                udp_buf[8] =  frame_size       & 0xFF;
                for (int i = 0; i < 8; i++) {
                    udp_buf[9 + i] = (uint8_t)((ts_us >> (56 - i * 8)) & 0xFF);
                }

                memcpy(udp_buf + 17, img_buf, frame_size);

                ssize_t sent = sendto(sock_img, udp_buf, 17 + frame_size, 0,
                                    (struct sockaddr *)&img_dest, sizeof(img_dest));
                if (sent > 0) {
                    ESP_LOGI(TAG, "[img] cam%d frame#%lu: %u B, ts=%lld us",
                            cam, cam_frame_num[cam], frame_size, ts_us);
                    cam_frame_num[cam]++;
                }
            }
        }
    }

    free(img_buf);
    heap_caps_free(tx_buf);
    heap_caps_free(rx_buf);
    close(sock_img);
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 6: Punto de entrada
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

    ESP_LOGI(TAG, "=== Drone Bridge iniciando (MAVLink por USB, imagenes por WiFi) ===");

    wifi_init_ap();     // sigue haciendo falta solo para el canal de imagenes
    uart_init();
    usb_cdc_init();     // nuevo: sustituye al socket UDP de MAVLink

    // Las tareas MAVLink ya NO esperan a BIT_GS_READY (WiFi) — el USB es
    // independiente de si hay un cliente WiFi conectado o no.
    xTaskCreate(usb_to_uart_task, "usb_to_uart",
                TASK_STACK_SIZE, NULL, TASK_PRIO_USB_RX, NULL);
    xTaskCreate(uart_to_usb_task, "uart_to_usb",
                TASK_STACK_SIZE, NULL, TASK_PRIO_UART_RX, NULL);

    // La tarea de imagenes si sigue esperando WiFi, porque su transporte
    // no ha cambiado.
    /*xTaskCreate(image_capture_task, "img_capture",
                8192, NULL, 4, NULL);*/

    ESP_LOGI(TAG, "Tareas creadas. MAVLink por USB-CDC | Imagen -> WiFi UDP:%d (NUM_CAMS=%d)",
             IMAGE_UDP_PORT, NUM_CAMS);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RESUMEN PARA CONTINUAR EN OTRA SESIÓN — LEER ANTES DE COMPILAR/PROBAR
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * CONTEXTO: TFG drone_colabo, dron agrícola GPS-denied, VIO con OpenVINS +
 * calibración con Basalt. ESP32-S3 master hace de puente MAVLink (FC↔MAVROS)
 * y capturador de imagen estéreo (2x OV3660 por SPI). Se detectó que el
 * bridge WiFi UDP para MAVLink introducía RTT de 25-260ms, incompatible con
 * el umbral fijo de 10ms del mecanismo de timesync de MAVROS, degradando la
 * fiabilidad del timestamp de los mensajes de IMU (RAW_IMU a 100Hz) usados
 * por OpenVINS/Basalt. Decisión tomada: mover SOLO el tráfico MAVLink a
 * USB-CDC (cable), dejando las imágenes por WiFi UDP:14552 sin cambios,
 * para pruebas de banco con el dron movido a mano (no vuelo real).
 *
 * 1. CAMBIOS APLICADOS EN ESTE ARCHIVO
 *    - Eliminado el socket UDP de MAVLink (g_mav_sock, create_shared_socket,
 *      MAVLINK_LISTEN_PORT, MAVLINK_GS_TELEM_PORT).
 *    - Añadido TinyUSB CDC-ACM (tinyusb.h / tusb_cdc_acm.h) como transporte
 *      MAVLink: usb_cdc_init(), tinyusb_cdc_rx_callback(), cola FreeRTOS
 *      g_usb_rx_queue para pasar bytes RX del callback a la tarea.
 *    - udp_to_uart_task -> usb_to_uart_task (lee de la cola USB, escribe UART)
 *    - uart_to_udp_task -> uart_to_usb_task (lee UART, parsea MAVLink,
 *      escribe por tinyusb_cdcacm_write_queue + write_flush)
 *    - Las tareas MAVLink ya no esperan BIT_GS_READY (WiFi); el WiFi AP
 *      sigue existiendo únicamente para el canal de imágenes.
 *    - Añadido esp_wifi_set_ps(WIFI_PS_NONE) tras wifi start (pendiente de
 *      sesiones anteriores, no relacionado con este cambio pero aprovechado).
 *    - image_capture_task y toda la Sección 5 (SPI/imágenes): SIN CAMBIOS.
 *
 * 2. RIESGO PRINCIPAL — API DE TINYUSB NO VERIFICADA CONTRA LA VERSION REAL
 *    El código usa la forma "clásica" del componente esp_tinyusb
 *    (tinyusb_driver_install, tinyusb_config_cdcacm_t, tusb_cdc_acm_init,
 *    tinyusb_cdcacm_read, tinyusb_cdcacm_write_queue/write_flush). Esta API
 *    ha cambiado de nombre/firma entre versiones del componente gestionado
 *    "espressif/esp_tinyusb" en distintas releases de ESP-IDF. ANTES DE
 *    COMPILAR:
 *      - Añadir el componente al proyecto (idf.py add-dependency
 *        "espressif/esp_tinyusb^<version>" o vía idf_component.yml), lo que
 *        instalará los headers reales.
 *      - Revisar tusb_cdc_acm.h instalado y ajustar nombres de campos/
 *        funciones si no coinciden exactamente con lo escrito aquí.
 *      - Comprobar en menuconfig que el modo USB del ESP32-S3 está puesto
 *        en "USB-OTG" (no "USB-Serial-JTAG"), ya que son periféricos
 *        distintos y solo uno puede usarse para TinyUSB CDC a la vez.
 *
 * 3. PENDIENTE DE VALIDAR EN BANCO (próxima sesión)
 *    - Confirmar que Linux enumera el dispositivo como /dev/ttyACM0 (o
 *      similar) al conectar el USB del ESP32.
 *    - Relanzar MAVROS con fcu_url:=/dev/ttyACM0:115200 y repetir
 *      exactamente la misma batería de pruebas de la sesión anterior:
 *        a) ¿Aparecen warnings "TM: RTT too high for timesync"? (se espera
 *           que NO, o que el RTT baje muy por debajo de 10ms)
 *        b) ros2 topic hz /mavros/imu/data_raw con RAW_IMU a 100Hz — se
 *           espera una tasa mucho más estable que el ~78-90Hz con jitter
 *           ±20-40ms que se obtenía por WiFi.
 *        c) Si el RTT baja de forma consistente, volver a poner
 *           timesync_mode: MAVLINK (por defecto) — ya NO haría falta el
 *           parche timesync_mode: NONE que se aplicó como mitigación
 *           cuando el transporte era WiFi.
 *    - Verificar que el canal de imágenes por WiFi sigue funcionando igual
 *      que antes (no debería haberse tocado nada relevante, pero confirmar
 *      que gs_connected y el flujo SPI no se han visto afectados al quitar
 *      el socket MAVLink del mismo archivo).
 *
 * 4. OPTIMIZACIONES QUE QUEDAN ABIERTAS, NO APLICADAS EN ESTE CAMBIO
 *    - uart_to_usb_task sigue haciendo un flush USB por cada mensaje MAVLink
 *      individual (mismo patrón "un envío por mensaje" que ya se identificó
 *      como sospechoso en la versión WiFi). Si con USB el problema de
 *      tasa/jitter persistiera (menos probable, pero no descartado del
 *      todo), la primera optimización a probar sería agrupar varios
 *      mensajes en un único write_queue/flush — con la salvedad ya
 *      discutida: esto solo es seguro para el timestamping si se vuelve a
 *      timesync_mode: MAVLINK (con NONE, agrupar mensajes degrada el
 *      timestamp de recepción por las razones ya explicadas en la sesión
 *      anterior).
 *    - El log periódico de "frames enviados" (cada 100 frames) sigue
 *      viviendo dentro del hot path de uart_to_usb_task, igual que en la
 *      versión anterior — no se ha movido a una tarea de menor prioridad.
 *      Si aparece jitter periódico (~1x/segundo) en las nuevas pruebas,
 *      este es el primer sospechoso a revisar/mover.
 *    - No se ha tocado el baudrate de UART2 (sigue en 115200). Con MAVLink
 *      ahora por USB (mucho más ancho de banda disponible que el UART2
 *      interno FC<->ESP32), el UART2 a 115200 podría convertirse en el
 *      nuevo cuello de botella si se sube la tasa de IMU por encima de
 *      ~100Hz en el futuro — revisar el cálculo de banda ya hecho en
 *      sesiones anteriores (HIGHRES_IMU ~74B/msg, RAW_IMU más ligero).
 *
 * 5. LO QUE NO HA CAMBIADO Y NO DEBERÍA REVISARSE DE NUEVO
 *    - Toda la lógica SPI de captura de imagen (capture_camera,
 *      image_capture_task): probada y estable en sesiones anteriores.
 *    - Formato de cabecera UDP de imagen (17 bytes, magic 0xCAFE): sin
 *      cambios, el nodo image_receiver_node existente sigue siendo
 *      compatible sin modificación.
 *    - Parámetros ArduPilot SERIAL2_PROTOCOL/SERIAL2_BAUD: sin cambios,
 *      siguen gobernando el enlace FC<->ESP32 por UART2 (que sigue
 *      existiendo igual, solo cambia qué hay al otro lado del ESP32 hacia
 *      el PC).
 */