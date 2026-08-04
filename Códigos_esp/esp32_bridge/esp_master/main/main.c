/**
 * @file main.c
 * @brief ESP32-S3 master — Puente MAVLink UDP<->UART + captura de imagen estereo por SPI
 *
 * Arquitectura de red:
 *   ESP32  ->  AP WiFi "DroneNet"  ->  IP 192.168.4.1
 *   GS     ->  cliente WiFi        ->  IP 192.168.4.2  (fija por DHCP)
 *
 * Flujo MAVLink:
 *   GS (MAVROS) --UDP:14550--> ESP32 --UART2--> SpeedyBee F405
 *   GS (MAVROS) <--UDP:14551-- ESP32 <-UART2-- SpeedyBee F405
 *
 * Flujo de imagen (independiente de MAVROS, socket propio):
 *   Slaves (SPI, trigger pautado) -> ESP32 master -> UDP:14552 -> nodo ROS 2 dedicado
 *
 * capture_camera() incluye reintento por chunk y ACK_DONE garantizado
 * (incluso si el frame falla), para que el slave nunca quede con un
 * frame "atascado" sin confirmar tras un fallo puntual de lectura.
 *
 * NUM_CAMS controla cuantas camaras se leen en el ciclo de captura.
 */

#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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

#define MAVLINK_LISTEN_PORT   14550
#define MAVLINK_GS_TELEM_PORT 14551

#define UART_PORT_NUM      UART_NUM_2
#define UART_TX_GPIO       17
#define UART_RX_GPIO       16
#define UART_BAUD_RATE     115200
#define UART_DRIVER_BUF    4096

#define UDP_RX_BUF_SIZE    1024
#define UART_RX_BUF_SIZE   1024

#define TASK_STACK_SIZE    4096
#define TASK_PRIO_UDP_RX   5
#define TASK_PRIO_UART_RX  5

/* ═══════════════════════════════════════════════════════════════════════════
 * CONFIGURACIÓN — CAPTURA DE IMAGEN
 * ═══════════════════════════════════════════════════════════════════════════ */

#define NUM_CAMS   2     // pasar a 2 cuando la prueba con 1 camara este validada
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
#define MAX_FRAME_BYTES     (60 * 1024)   // subido a 60KB para VGA, antes 32KB para QVGA
#define IMG_HEADER_SIZE     21   // antes 17; +4 bytes de cycle_id
#define SPI_PHASE_DELAY_MS  10             
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

static int g_mav_sock = -1;

static const int            cam_cs_pins[MAX_CAMS] = { PIN_MASTER_CS0, PIN_MASTER_CS1 };
static spi_device_handle_t  cam_dev[MAX_CAMS];
static uint32_t             cam_frame_num[MAX_CAMS] = { 0, 0 };
static uint32_t             g_cycle_id = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 1: WiFi AP
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
        ESP_LOGI(TAG, "GS conectada: %s (AID=%d)", mac_str, ev->aid);
        gs_connected = true;
        xEventGroupSetBits(wifi_event_group, BIT_GS_READY);

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)event_data;
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(ev->mac));
        ESP_LOGW(TAG, "GS desconectada: %s", mac_str);
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

    ESP_LOGI(TAG, "AP iniciado  SSID='%s'  IP=%s", WIFI_AP_SSID, AP_IP_STR);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 2: UART hacia SpeedyBee
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
 * SECCIÓN 3: Socket UDP compartido (MAVLink)
 * ═══════════════════════════════════════════════════════════════════════════ */

static int create_shared_socket(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() fallo: errno=%d", errno);
        return -1;
    }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(MAVLINK_LISTEN_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() fallo: errno=%d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Socket MAVLink compartido bindeado a :%d", MAVLINK_LISTEN_PORT);
    return sock;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 4: Tareas MAVLink
 * ═══════════════════════════════════════════════════════════════════════════ */

static void udp_to_uart_task(void *arg)
{
    ESP_LOGI(TAG, "[udp->uart] esperando GS...");
    xEventGroupWaitBits(wifi_event_group, BIT_GS_READY,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "[udp->uart] escuchando en socket compartido :%d",
             MAVLINK_LISTEN_PORT);

    uint8_t buf[UDP_RX_BUF_SIZE];

    while (true) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);

        int len = recvfrom(g_mav_sock, buf, sizeof(buf), 0,
                           (struct sockaddr *)&src, &src_len);

        if (len < 0) {
            ESP_LOGW(TAG, "[udp->uart] recvfrom error: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (len > 0) {
            uart_write_bytes(UART_PORT_NUM, buf, len);
        }
    }

    vTaskDelete(NULL);
}

static void uart_to_udp_task(void *arg)
{
    ESP_LOGI(TAG, "[uart->udp] esperando GS...");
    xEventGroupWaitBits(wifi_event_group, BIT_GS_READY,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    uint8_t raw[UART_RX_BUF_SIZE];
    mavlink_message_t msg;
    mavlink_status_t  mav_status;
    uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];

    struct sockaddr_in gs_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(MAVLINK_GS_TELEM_PORT),
        .sin_addr.s_addr = inet_addr(GS_IP_STR),
    };

    ESP_LOGI(TAG, "[uart->udp] enviando MAVLink a %s:%d (origen :%d)",
             GS_IP_STR, MAVLINK_GS_TELEM_PORT, MAVLINK_LISTEN_PORT);

    uint32_t frames_sent = 0;
    uint32_t frames_log  = 0;

    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, raw, sizeof(raw),
                                  pdMS_TO_TICKS(20));

        for (int i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, raw[i], &msg, &mav_status)) {
                uint16_t frame_len = mavlink_msg_to_send_buffer(mav_buf, &msg);

                if (gs_connected) {
                    sendto(g_mav_sock, mav_buf, frame_len, 0,
                           (struct sockaddr *)&gs_addr, sizeof(gs_addr));
                    frames_sent++;
                }
            }
        }

        if (frames_sent - frames_log >= 100) {
            ESP_LOGI(TAG, "[uart->udp] %lu frames MAVLink enviados", frames_sent);
            frames_log = frames_sent;
        }

        if (len == 0) {
            taskYIELD();
        }
    }

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 5: Captura de imagen (SPI, trigger pautado, N camaras)
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

/**
 * Lee un frame completo de una camara. Reintenta cada chunk individual
 * hasta CHUNK_MAX_RETRIES veces antes de descartar el frame. SIEMPRE
 * envia CMD_ACK_DONE al final, incluso si el frame fallo — evita que
 * el slave quede con g_frame_ready atascado en true (causa raiz de los
 * "trigger ignorado" en cadena observados anteriormente).
 */
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

    // ACK_DONE siempre, exito o fallo, para no dejar el slave atascado
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
    ESP_LOGI(TAG, "[img] esperando GS...");
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
    uint8_t *udp_buf = malloc(IMG_HEADER_SIZE + MAX_FRAME_BYTES);   // cabecera + imagen en un unico buffer
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
        
        const uint32_t this_cycle = g_cycle_id++;

        for (int cam = 0; cam < NUM_CAMS; cam++) {
            uint16_t frame_size;
            int64_t  ts_us;

            bool ok = capture_camera(cam_dev[cam], (uint8_t)cam,
                                      tx_buf, rx_buf, img_buf,
                                      &frame_size, &ts_us);
            if (!ok) continue;

            if (gs_connected) {
                // Cabecera escrita directamente en los primeros 21 bytes de udp_buf
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
                udp_buf[17] = (this_cycle >> 24) & 0xFF;
                udp_buf[18] = (this_cycle >> 16) & 0xFF;
                udp_buf[19] = (this_cycle >>  8) & 0xFF;
                udp_buf[20] =  this_cycle        & 0xFF;
                // Imagen justo despues de la cabecera, en el mismo buffer
                memcpy(udp_buf + IMG_HEADER_SIZE, img_buf, frame_size);

                // Un unico sendto: cabecera+imagen viajan como un solo datagrama UDP,
                // eliminando cualquier posibilidad de que se reordenen entre si
                ssize_t sent = sendto(sock_img, udp_buf, IMG_HEADER_SIZE + frame_size, 0,
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

    ESP_LOGI(TAG, "=== Drone Bridge iniciando ===");

    wifi_init_ap();
    uart_init();

    xEventGroupWaitBits(wifi_event_group, BIT_GS_READY,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    g_mav_sock = create_shared_socket();
    if (g_mav_sock < 0) {
        ESP_LOGE(TAG, "No se pudo crear el socket compartido, abortando");
        return;
    }

    xTaskCreate(udp_to_uart_task, "udp_to_uart",
                TASK_STACK_SIZE, NULL, TASK_PRIO_UDP_RX, NULL);
    xTaskCreate(uart_to_udp_task, "uart_to_udp",
                TASK_STACK_SIZE, NULL, TASK_PRIO_UART_RX, NULL);
    xTaskCreate(image_capture_task, "img_capture",
                8192, NULL, 4, NULL);

    ESP_LOGI(TAG, "Tareas creadas. MAVLink :%d -> GS:%d | Imagen -> GS:%d (NUM_CAMS=%d)",
             MAVLINK_LISTEN_PORT, MAVLINK_GS_TELEM_PORT, IMAGE_UDP_PORT, NUM_CAMS);
}