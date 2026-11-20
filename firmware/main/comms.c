
#include "comms.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "app.h"
#include "cc_proto3.h"

_Static_assert(CC3_TM_SIZE <= CC_MAX_PAYLOAD, "TM_STATE3 exceeds frame payload");

static const char *TAG = "comms";

volatile uint8_t g_state_hz = 20;
volatile uint8_t g_vision_hz = 20;
volatile uint8_t g_img_every = 10;

static int s_sock = -1;
static struct sockaddr_in s_client;

static volatile bool s_have_client = false;
static volatile bool s_ip_ok = false;

static SemaphoreHandle_t s_tx_mtx;
static uint8_t s_seq = 0;
static uint8_t s_img_id = 0;

/* ---------------------------------------------------
 * Eventos Wi-Fi
 * --------------------------------------------------- */

static void on_event(
    void *arg,
    esp_event_base_t base,
    int32_t id,
    void *data)
{
    if (base == WIFI_EVENT &&
        id == WIFI_EVENT_STA_START) {

        esp_wifi_connect();

    } else if (base == WIFI_EVENT &&
               id == WIFI_EVENT_STA_DISCONNECTED) {

        s_ip_ok = false;
        s_have_client = false;

        ESP_LOGW(TAG,
                 "Wi-Fi desligado, a tentar novamente");

        esp_wifi_connect();

    } else if (base == IP_EVENT &&
               id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *e =
            (ip_event_got_ip_t *)data;

        ESP_LOGI(
            TAG,
            "IP " IPSTR " - UDP %d",
            IP2STR(&e->ip_info.ip),
            CONFIG_CUBE_GS_PORT
        );

        s_ip_ok = true;
    }
}

/* ---------------------------------------------------
 * Inicializacao Wi-Fi
 * --------------------------------------------------- */

void comms_init(void)
{
    
    s_tx_mtx = xSemaphoreCreateMutex();
    configASSERT(s_tx_mtx != NULL);

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &on_event,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &on_event,
            NULL
        )
    );

    wifi_config_t wc = {0};

    strncpy(
        (char *)wc.sta.ssid,
        CONFIG_CUBE_WIFI_SSID,
        sizeof(wc.sta.ssid) - 1
    );

    strncpy(
        (char *)wc.sta.password,
        CONFIG_CUBE_WIFI_PASS,
        sizeof(wc.sta.password) - 1
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA)
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(WIFI_IF_STA, &wc)
    );

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(
        esp_wifi_set_ps(WIFI_PS_NONE)
    );
}

/* ---------------------------------------------------
 * Envio de tramas TM
 * --------------------------------------------------- */

static void send_frame(
    uint8_t type,
    const uint8_t *pl,
    uint8_t len)
{
    if (len > CC_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload TM demasiado grande");
        return;
    }

    if (!s_tx_mtx) {
        return;
    }

    uint8_t out[
        CC_MAX_PAYLOAD + CC_FRAME_OVERHEAD
    ];

    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);

    if (!s_have_client || s_sock < 0 || !s_ip_ok) {
        xSemaphoreGive(s_tx_mtx);
        return;
    }

    size_t n = cc_frame_pack(
        out,
        type,
        ++s_seq,
        pl,
        len
    );

    if (n > 0 && n <= sizeof(out)) {
        int sent = sendto(
            s_sock,
            out,
            n,
            0,
            (struct sockaddr *)&s_client,
            sizeof(s_client)
        );

        if (sent < 0 || (size_t)sent != n) {
            ESP_LOGW(TAG, "Falha no envio UDP");
        }
    }

    xSemaphoreGive(s_tx_mtx);
}

/* ---------------------------------------------------
 * Telemetria de visao
 * --------------------------------------------------- */

void comms_send_vision(
    const cc_vision_t *v,
    uint32_t proc_us)
{
    if (!v) {
        return;
    }

    uint8_t pl[64];

    size_t n = cc_pack_tm_vision(
        pl,
        v,
        proc_us
    );

    if (n <= sizeof(pl)) {
        send_frame(TM_VISION, pl, (uint8_t)n);
    }
}

/* ---------------------------------------------------
 * Envio de imagem
 * --------------------------------------------------- */

void comms_send_image(const uint8_t *gray)
{
    if (!gray) {
        return;
    }

    uint8_t pl[CC_MAX_PAYLOAD];

    s_img_id++;

    for (int c = 0; c < CC_IMG_NCHUNKS; c++) {

        size_t n = cc_pack_tm_img_chunk(
            pl,
            gray,
            s_img_id,
            (uint8_t)c
        );

        if (n <= sizeof(pl)) {
            send_frame(
                TM_IMG,
                pl,
                (uint8_t)n
            );
        }

        if ((c & 3) == 3) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

/* ---------------------------------------------------
 * Processamento de telecomandos
 *
 * Comandos legados e de tres eixos usam o mesmo decoder do Core.
 * --------------------------------------------------- */

static void handle_tc(
    uint8_t type,
    uint8_t seq,
    const uint8_t *p,
    uint8_t len)
{
    uint8_t status = ACK_OK;

    if (type == TC_PING) {

        if (len != 0) {
            status = ACK_BAD_LEN;
        }

    } else if (type == TC_STREAM_CFG) {

        if (len != 3) {
            status = ACK_BAD_LEN;

        } else {
            g_state_hz = p[0];
            g_vision_hz = p[1];
            g_img_every = p[2];
        }

    } else {

        CORE_LOCK();
        status = cc3_apply_tc(&g_core, type, p, len);
        CORE_UNLOCK();
    }

    uint8_t ack[3];

    size_t n = cc_pack_tm_ack(
        ack,
        type,
        seq,
        status
    );

    if (n <= sizeof(ack)) {
        send_frame(
            TM_ACK,
            ack,
            (uint8_t)n
        );
    }
}

/* ---------------------------------------------------
 * Abertura do socket UDP
 * --------------------------------------------------- */

static void open_socket(void)
{
    int s = socket(
        AF_INET,
        SOCK_DGRAM,
        IPPROTO_IP
    );

    if (s < 0) {
        ESP_LOGE(TAG, "Falha ao criar socket");
        return;
    }

    struct sockaddr_in a = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_CUBE_GS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(
            s,
            (struct sockaddr *)&a,
            sizeof(a)
        ) != 0) {

        ESP_LOGE(TAG, "Falha no bind UDP");
        close(s);
        return;
    }

    s_sock = s;
}

/* ---------------------------------------------------
 * Tarefa principal de comunicacoes
 * --------------------------------------------------- */

void comms_task(void *arg)
{
    uint64_t next_state = 0;

    for (;;) {

        if (s_sock < 0 && s_ip_ok) {
            open_socket();
        }

        if (s_sock >= 0) {

            uint8_t rx[512];

            struct sockaddr_in from;
            socklen_t fl = sizeof(from);

            int n = recvfrom(
                s_sock,
                rx,
                sizeof(rx),
                MSG_DONTWAIT,
                (struct sockaddr *)&from,
                &fl
            );

            if (n > 0) {

                cc_parser_t parser;
                cc_parser_init(&parser);

                bool valid = false;

                for (int i = 0; i < n; i++) {

                    if (cc_parser_feed(
                            &parser,
                            rx[i]
                        )) {

                        if (i != n - 1) {
                            valid = false;
                            break;
                        }

                        valid = true;
                    }
                }

                /*
                 * Uma trama completa por datagrama.
                 *
                 * Nao limitar os comandos a TC_RECAL:
                 * os novos tipos podem estar acima
                 * desse identificador.
                 *
                 * A validacao semantica e feita
                 * por cc3_apply_tc().
                 */
                if (
                    valid &&
                    n == parser.len + CC_FRAME_OVERHEAD &&
                    rx[0] == 0xA5 &&
                    rx[1] == 0x5A &&
                    parser.type >= TC_PING
                ) {

                    xSemaphoreTake(
                        s_tx_mtx,
                        portMAX_DELAY
                    );

                    s_client = from;
                    s_have_client = true;

                    xSemaphoreGive(s_tx_mtx);

                    handle_tc(
                        parser.type,
                        parser.seq,
                        parser.payload,
                        parser.len
                    );
                }
            }

            /* Telemetria periodica */
            uint64_t now =
                (uint64_t)esp_timer_get_time();

            if (
                g_state_hz &&
                now >= next_state
            ) {

                cc_tlm_t t;
                cc3_tlm_t t3;

                /* Um unico snapshot para os dois formatos de telemetria. */
                CORE_LOCK();
                cc3_get_legacy_tlm(&g_core, &t);
                cc3_get_tlm(&g_core, &t3);
                CORE_UNLOCK();

                uint8_t pl[CC3_TM_SIZE];

                size_t payload_len =
                    cc_pack_tm_state(pl, &t);

                if (payload_len <= sizeof(pl)) {

                    send_frame(
                        TM_STATE,
                        pl,
                        (uint8_t)payload_len
                    );
                }

                payload_len = cc3_pack_tm(pl, sizeof(pl), &t3);
                if (payload_len == CC3_TM_SIZE) {
                    send_frame(TM_STATE3, pl, (uint8_t)payload_len);
                }

                next_state =
                    now + 1000000ULL / g_state_hz;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
