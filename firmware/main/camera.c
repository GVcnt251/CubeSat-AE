#include "camera.h"
#include <string.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "app.h"
#include "comms.h"
#include "cc_proto.h"
#include "target_detect.h"

static const char *TAG = "camera";
static cc_det_ws_t s_ws;           /* workspace só desta tarefa (o do núcleo não é usado no firmware) */

/* Seeed XIAO ESP32S3 Sense */
static camera_config_t cam_cfg(void)
{
    camera_config_t c = {
        .pin_pwdn = -1, .pin_reset = -1, .pin_xclk = 10, .pin_sccb_sda = 40, .pin_sccb_scl = 39,
        .pin_d7 = 48, .pin_d6 = 11, .pin_d5 = 12, .pin_d4 = 14, .pin_d3 = 16, .pin_d2 = 18, .pin_d1 = 17, .pin_d0 = 15,
        .pin_vsync = 38, .pin_href = 47, .pin_pclk = 13,
        .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE, .frame_size = FRAMESIZE_QVGA,     /* 320x240, 8 bit */
        .jpeg_quality = 12, .fb_count = 2, .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,                                       /* sempre a imagem mais recente */
    };
    return c;
}

void camera_task(void *arg)
{
    camera_config_t cfg = cam_cfg();
    if (esp_camera_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init falhou: o cubo fica SEM visão (modos IDLE/WHEEL/RATE/HEADING continuam a funcionar)");
        vTaskDelete(NULL);
        return;
    }
    cc_det_cfg_t det_cfg;
    CORE_LOCK(); det_cfg = g_core.cfg.det; CORE_UNLOCK();

    uint32_t n = 0;
    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        if (fb->format != PIXFORMAT_GRAYSCALE || fb->width != CC_IMG_W || fb->height != CC_IMG_H) {
            ESP_LOGE(TAG, "formato inesperado %dx%d fmt=%d", (int)fb->width, (int)fb->height, (int)fb->format);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        uint64_t t0 = (uint64_t)esp_timer_get_time();
        uint64_t t_cap = t0 - (uint64_t)CONFIG_CUBE_CAM_LATENCY_MS * 1000ULL;

        cc_det_t det;
        cc_detect(&det_cfg, &s_ws, fb->buf, &det);                 /* pesado: fora do mutex */
        uint32_t proc_us = (uint32_t)((uint64_t)esp_timer_get_time() - t0);

        cc_vision_t vis;
        CORE_LOCK();
        cc_on_detection(&g_core, t_cap, (uint64_t)esp_timer_get_time(), &det, &vis);   /* leve */
        CORE_UNLOCK();

        if (g_vision_hz) comms_send_vision(&vis, proc_us);
        if (g_img_every && (++n % g_img_every) == 0) comms_send_image(fb->buf);
        esp_camera_fb_return(fb);
    }
}
