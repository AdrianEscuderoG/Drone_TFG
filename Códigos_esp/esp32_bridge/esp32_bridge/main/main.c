/**
 * @file main.c
 * @brief ESP32-S3 master — Puente MAVLink UDP ↔ UART
 *
 * Arquitectura de red:
 *   ESP32  →  AP WiFi "DroneNet"  →  IP 192.168.4.1
 *   GS     →  cliente WiFi        →  IP 192.168.4.2  (fija por DHCP)
 *
 * Flujo de datos:
 *   GS (MAVROS) ──UDP:14550──► ESP32 ──UART2──► SpeedyBee F405
 *   GS (MAVROS) ◄──UDP:14551── ESP32 ◄─UART2── SpeedyBee F405
 *
 * IMPORTANTE — el socket compartido en el ESP32 esta bindeado a 14550
 * (ahi escucha comandos), pero la TELEMETRIA debe enviarse al puerto
 * 14551 porque es donde MAVROS realmente escucha (segun fcu_url).
 * El bind solo fija el puerto de ORIGEN; el destino de cada sendto()
 * se especifica aparte y NO tiene por que coincidir con el de bind.
 *
 * BUG QUE CORRIGIO ESTA VERSION:
 *   Una version anterior unifico erroneamente bind y destino en un
 *   unico puerto 14550, lo que rompio la recepcion de telemetria en
 *   MAVROS (escuchaba en 14551, nunca le llegaba nada) y por tanto
 *   nunca se completaba el handshake CON: Got HEARTBEAT.
 *
 * Configuración MAVROS en la GS:
 *   fcu_url:=udp://:14551@192.168.4.1:14550
 *
 * Pines UART2 (ESP32-S3 master):
 *   TX → GPIO 17  (conectado a RX de SpeedyBee UART2)
 *   RX → GPIO 16  (conectado a TX de SpeedyBee UART2)
 *
 * Parámetros ArduPilot requeridos:
 *   SERIAL2_PROTOCOL = 2   (MAVLink 2)
 *   SERIAL2_BAUD     = 115 (115200)
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
#include "common/mavlink.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * CONFIGURACIÓN
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

// Puerto donde el ESP32 escucha comandos de MAVROS (bind del socket)
#define MAVLINK_LISTEN_PORT  14550
// Puerto donde MAVROS escucha telemetria (segun el fcu_url: udp://:14551@...)
// IMPORTANTE: este es el puerto de DESTINO de la telemetria, no el de bind.
// El socket sigue bindeado a MAVLINK_LISTEN_PORT (su puerto de origen sale
// siempre como 14550), pero el sendto() debe apuntar a 14551 porque es
// donde MAVROS realmente escucha segun su fcu_url.
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
 * VARIABLES GLOBALES
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *TAG = "bridge";

static const uint8_t gs_mac[6] = GS_MAC_ADDR;
static volatile bool gs_connected = false;

static EventGroupHandle_t wifi_event_group;
#define BIT_GS_READY  BIT0

// Socket UDP compartido entre las dos tareas MAVLink.
// Se crea una sola vez en app_main y se pasa a ambas tareas.
static int g_mav_sock = -1;

// Mutex para proteger el uso concurrente del socket entre las dos tareas
// (sendto desde uart_to_udp_task, recvfrom desde udp_to_uart_task).
// En lwIP sendto/recvfrom sobre el mismo fd desde distintas tareas es
// seguro a nivel de socket, pero protegemos por claridad y portabilidad.
static SemaphoreHandle_t g_sock_mutex;

// Histograma de mensajes MAVLink parseados desde el UART, indexado por msgid.
// Sirve para saber QUE tipos de mensaje llegan del FC, no solo cuantos en total.
static uint32_t g_msgid_counts[256] = {0};
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
            ESP_LOGW(TAG, "MAC no autorizada: %s — rechazando", mac_str);
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
 * SECCIÓN 3: Socket UDP compartido
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Crear y bindear el socket UDP compartido a MAVLINK_LISTEN_PORT.
 *
 * Este es EL FIX: antes uart_to_udp_task creaba su propio socket sin
 * bind, lo que le daba un puerto de origen efimero aleatorio. MAVROS
 * detecta el puerto de origen del primer paquete recibido y empieza a
 * responder ahi (comportamiento normal de auto-discovery), por lo que
 * los comandos GS->FC se enviaban a un puerto que el ESP32 nunca
 * escuchaba.
 *
 * Con un unico socket bindeado a 14550 usado tanto para recvfrom como
 * para sendto, el puerto de origen de la telemetria SIEMPRE es 14550,
 * coincidiendo con el puerto que el ESP32 efectivamente escucha.
 */
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
 * SECCIÓN 4: Tareas FreeRTOS del puente
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Tarea: GS → UDP → UART → SpeedyBee  (comandos al FC)
 *
 * Usa el socket compartido g_mav_sock solo para recvfrom.
 */
static void udp_to_uart_task(void *arg)
{
    ESP_LOGI(TAG, "[udp→uart] esperando GS...");
    xEventGroupWaitBits(wifi_event_group, BIT_GS_READY,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "[udp→uart] escuchando en socket compartido :%d",
             MAVLINK_LISTEN_PORT);

    uint8_t buf[UDP_RX_BUF_SIZE];

    while (true) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);

        // recvfrom bloqueante: la tarea duerme aqui sin consumir CPU.
        // No necesita mutex porque recvfrom y sendto sobre el mismo
        // socket UDP desde distintas tareas no interfieren entre si
        // en lwIP (no comparten estado mutable relevante aqui).
        int len = recvfrom(g_mav_sock, buf, sizeof(buf), 0,
                           (struct sockaddr *)&src, &src_len);

        if (len < 0) {
            ESP_LOGW(TAG, "[udp→uart] recvfrom error: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (len > 0) {
            ESP_LOGI(TAG, "[udp→uart] %d bytes de %s:%d", len,
                     inet_ntoa(src.sin_addr), ntohs(src.sin_port));
            uart_write_bytes(UART_PORT_NUM, buf, len);
        }
    }

    vTaskDelete(NULL);
}

/**
 * Tarea: SpeedyBee → UART → parser MAVLink → UDP → GS  (telemetria)
 *
 * Usa el socket compartido g_mav_sock solo para sendto, con destino fijo
 * GS_IP_STR:MAVLINK_GS_TELEM_PORT (14551). El socket sigue bindeado a 14550, el
 * paquete sale con puerto de origen 14550 — exactamente lo que MAVROS
 * espera segun el fcu_url configurado.
 */
static void uart_to_udp_task(void *arg)
{
    ESP_LOGI(TAG, "[uart→udp] esperando GS...");
    xEventGroupWaitBits(wifi_event_group, BIT_GS_READY,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    uint8_t raw[UART_RX_BUF_SIZE];
    mavlink_message_t msg;
    mavlink_status_t  mav_status;
    uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];

    // Destino: puerto donde MAVROS realmente escucha (14551 segun fcu_url),
    // NO el puerto de bind del socket (14550, que es solo el origen).
    struct sockaddr_in gs_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(MAVLINK_GS_TELEM_PORT),
        .sin_addr.s_addr = inet_addr(GS_IP_STR),
    };

    ESP_LOGI(TAG, "[uart→udp] enviando MAVLink a %s:%d (origen :%d)",
             GS_IP_STR, MAVLINK_GS_TELEM_PORT, MAVLINK_LISTEN_PORT);

    uint32_t frames_sent = 0;
    uint32_t frames_log  = 0;

    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, raw, sizeof(raw),
                                  pdMS_TO_TICKS(20));

        for (int i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, raw[i], &msg, &mav_status)) {

                // Contar por tipo de mensaje ANTES de intentar reenviarlo,
                // para separar "qué manda el FC" de "qué se pierde al reenviar".
                if (msg.msgid < 256) {
                    g_msgid_counts[msg.msgid]++;
                }

                uint16_t frame_len = mavlink_msg_to_send_buffer(mav_buf, &msg);

                if (gs_connected) {
                    int sent = sendto(g_mav_sock, mav_buf, frame_len, 0,
                                    (struct sockaddr *)&gs_addr, sizeof(gs_addr));
                    if (sent < 0) {
                        ESP_LOGW(TAG, "[uart→udp] sendto() fallo: errno=%d", errno);
                    } else {
                        frames_sent++;
                    }
                }
            }
        }

        // Log periodico de estadisticas, fuera del hot path
        if (frames_sent - frames_log >= 100) {
            ESP_LOGI(TAG, "[uart→udp] %lu frames MAVLink enviados", frames_sent);
            frames_log = frames_sent;

            // Volcado del histograma: solo los IDs que han aparecido al menos una vez
            for (int id = 0; id < 256; id++) {
                if (g_msgid_counts[id] > 0) {
                    ESP_LOGI(TAG, "[uart→udp]   msgid %3d: %lu", id, g_msgid_counts[id]);
                }
            }
        }

        if (len == 0) {
            taskYIELD();
        }
    }

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SECCIÓN 5: Punto de entrada
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

    // Esperar a que la GS este conectada antes de crear el socket,
    // igual que hacian las tareas individualmente antes.
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

    ESP_LOGI(TAG, "Tareas creadas. Escuchando :%d, telemetria -> GS:%d",
             MAVLINK_LISTEN_PORT, MAVLINK_GS_TELEM_PORT);
}