#include "comms.h"
#include <stdbool.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "app.h"
#include "cc_proto.h"

static const char *TAG = "comms";

volatile uint8_t g_state_hz = 20, g_vision_hz = 20, g_img_every = 10;

static int s_sock = -1;
static struct sockaddr_in s_client;
static volatile bool s_have_client, s_ip_ok;
static SemaphoreHandle_t s_tx_mtx;
static uint8_t s_seq, s_img_id;

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_ip_ok = false;
        ESP_LOGW(TAG, "Wi-Fi desligado, a tentar de novo");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP " IPSTR " - ligar a estação de terra a esta morada, UDP %d", IP2STR(&e->ip_info.ip), CONFIG_CUBE_GS_PORT);
        s_ip_ok = true;
    }
}

void comms_init(void)
{
    s_tx_mtx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_event, NULL));
    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, CONFIG_CUBE_WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, CONFIG_CUBE_WIFI_PASS, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);                 /* sem poupança de energia: latência previsível */
}

static void send_frame(uint8_t type, const uint8_t *pl, uint8_t len)
{
    if (!s_have_client || s_sock < 0) return;
    uint8_t out[CC_MAX_PAYLOAD + CC_FRAME_OVERHEAD];
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    size_t n = cc_frame_pack(out, type, ++s_seq, pl, len);
    sendto(s_sock, out, n, 0, (struct sockaddr *)&s_client, sizeof(s_client));
    xSemaphoreGive(s_tx_mtx);
}

void comms_send_vision(const cc_vision_t *v, uint32_t proc_us)
{
    uint8_t pl[64];
    send_frame(TM_VISION, pl, (uint8_t)cc_pack_tm_vision(pl, v, proc_us));
}

void comms_send_image(const uint8_t *gray)
{
    uint8_t pl[CC_MAX_PAYLOAD];
    s_img_id++;
    for (int c = 0; c < CC_IMG_NCHUNKS; c++) {
        send_frame(TM_IMG, pl, (uint8_t)cc_pack_tm_img_chunk(pl, gray, s_img_id, (uint8_t)c));
        if ((c & 3) == 3) vTaskDelay(pdMS_TO_TICKS(2));    /* não afogar os buffers do Wi-Fi */
    }
}

static void handle_tc(uint8_t type, uint8_t seq, const uint8_t *p, uint8_t len)
{
    uint8_t status = ACK_OK;
    if (type == TC_PING) {
    if (len != 0) {
        status = 1;  // Erro: comprimento inválido
    }
} else if (type == TC_STREAM_CFG) {
    if (len != 3) {
        status = 1;  // Erro: comprimento inválido
    } else {
        g_state_hz = p[0];
        g_vision_hz = p[1];
        g_img_every = p[2];
    }
    } else {
        cc_cmd_t cmd;
        int r = cc_tc_to_cmd(type, p, len, &cmd);
        if (r == 1) { CORE_LOCK(); cc_command(&g_core, &cmd); CORE_UNLOCK(); }
        else if (r < 0) status = (uint8_t)(-r);
        else status = ACK_UNKNOWN;
    }
    uint8_t ack[3];
    send_frame(TM_ACK, ack, (uint8_t)cc_pack_tm_ack(ack, type, seq, status));
}

static void open_socket(void)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s < 0) return;
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(CONFIG_CUBE_GS_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(s, (struct sockaddr *)&a, sizeof(a)) != 0) { close(s); return; }
    s_sock = s;
}

void comms_task(void *arg)
{
    
    uint64_t next_state = 0;
    for (;;) {
        if (s_sock < 0 && s_ip_ok) open_socket();
        if (s_sock >= 0) {
            uint8_t rx[512];
            struct sockaddr_in from; socklen_t fl = sizeof(from);
            int n = recvfrom(s_sock, rx, sizeof(rx), MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
            
            if (n > 0) {
                cc_parser_t parser;
                cc_parser_init(&parser);

                bool valid = false;

                for (int i = 0; i < n; i++) {
                    if (cc_parser_feed(&parser, rx[i])) {
                        // A trama tem de terminar no último byte
                        if (i != n - 1) {
                            valid = false;
                            break;
                        }
                        valid = true;
                    }
                }

                // Só aceitar uma trama completa por datagrama
                if (valid &&
                    n == parser.len + 7 &&
                    rx[0] == 0xA5 &&
                    rx[1] == 0x5A &&
                    parser.type >= TC_PING &&
                    parser.type <= TC_RECAL) {

                    // Guardar o endereço apenas após validar a trama
                    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
                    s_client = from;
                    s_have_client = true;
                    xSemaphoreGive(s_tx_mtx);

                    // Processar o comando
                    handle_tc(parser.type, parser.seq,
                        parser.payload, parser.len);
                }
            }
            
            uint64_t now = (uint64_t)esp_timer_get_time();

            if (g_state_hz && now >= next_state) {
                cc_tlm_t t;

                CORE_LOCK();
                cc_get_tlm(&g_core, &t);
                CORE_UNLOCK();
                
                uint8_t pl[64];

                send_frame(TM_STATE, pl,
                    (uint8_t)cc_pack_tm_state(pl, &t));

                next_state = now + 1000000ULL / g_state_hz;
            }

        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
