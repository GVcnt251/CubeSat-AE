#include "bench_camera.h"

#include <inttypes.h>
#include <stdio.h>

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "sdkconfig.h"

#if !CONFIG_SCCB_HARDWARE_I2C_PORT1
#error "The bench camera uses SCCB port 1. The IMU uses port 0."
#endif

enum {
    FRAME_WIDTH = 160,
    FRAME_HEIGHT = 120,
    MAX_JPEG_BYTES = 12 * 1024,
    FRAME_PERIOD_MS = 50,
    CHUNK_BYTES = 384,
    CHUNK_PAUSE_MS = 1,
    STATUS_INTERVAL_US = 1000000,
};
static const char *TAG = "camera_usb";

static void publish_status(const char *status)
{
    printf("BENCH_CAMERA {\"schema\":1,\"status\":\"%s\"}\n", status);
}

static void publish_frame(const camera_fb_t *frame)
{
    const int64_t captured_us = (int64_t)frame->timestamp.tv_sec * 1000000 + frame->timestamp.tv_usec;
    const uint32_t checksum = esp_rom_crc32_le(0, frame->buf, frame->len);
    printf("BENCH_CAMERA {\"schema\":1,\"status\":\"frame\",\"sample_us\":%" PRIi64
           ",\"width\":%u,\"height\":%u,\"size\":%u,\"crc32\":%" PRIu32 ",\"chunk_size\":%u}\n",
           captured_us, FRAME_WIDTH, FRAME_HEIGHT, (unsigned)frame->len, checksum, CHUNK_BYTES);

    for (size_t offset = 0; offset < frame->len; offset += CHUNK_BYTES) {
        const size_t remaining = frame->len - offset;
        const size_t count = remaining < CHUNK_BYTES ? remaining : CHUNK_BYTES;
        unsigned char encoded[4 * ((CHUNK_BYTES + 2) / 3) + 1];
        size_t written;
        if (mbedtls_base64_encode(encoded, sizeof encoded, &written, frame->buf + offset, count) != 0) {
            return;
        }
        printf("BENCH_CAMERA_DATA %u %.*s\n", (unsigned)offset, (int)written, (const char *)encoded);
        // Let the IMU and CAN tasks publish between camera lines.
        vTaskDelay(pdMS_TO_TICKS(CHUNK_PAUSE_MS));
    }
}

static void camera_task(void *arg)
{
    (void)arg;
    publish_status("initializing");
    const camera_config_t config = {
        // Matches the OV5640 wiring used by the installed factory firmware.
        .pin_pwdn = -1, .pin_reset = -1, .pin_xclk = 10,
        .pin_sccb_sda = 40, .pin_sccb_scl = 39,
        .pin_d7 = 48, .pin_d6 = 11, .pin_d5 = 12, .pin_d4 = 14,
        .pin_d3 = 16, .pin_d2 = 18, .pin_d1 = 17, .pin_d0 = 15,
        .pin_vsync = 38, .pin_href = 47, .pin_pclk = 13,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_1, .ledc_channel = LEDC_CHANNEL_3,
        .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QQVGA,
        .jpeg_quality = 16, .fb_count = 2, .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_LATEST, .jpeg_buffer_size = MAX_JPEG_BYTES,
    };
    const esp_err_t init = esp_camera_init(&config);
    const sensor_t *sensor = init == ESP_OK ? esp_camera_sensor_get() : NULL;
    if (!sensor || sensor->id.PID != OV5640_PID) {
        if (init == ESP_OK) {
            esp_camera_deinit();
        }
        ESP_LOGW(TAG, "OV5640 unavailable: %s", esp_err_to_name(init == ESP_OK ? ESP_ERR_NOT_FOUND : init));
        for (;;) {
            publish_status("unavailable");
            vTaskDelay(pdMS_TO_TICKS(4000));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(250));
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_error_us = -STATUS_INTERVAL_US;
    for (;;) {
        camera_fb_t *frame = esp_camera_fb_get();
        const int64_t now = esp_timer_get_time();
        if (frame && frame->format == PIXFORMAT_JPEG && frame->width == FRAME_WIDTH &&
            frame->height == FRAME_HEIGHT && frame->len >= 4 && frame->len <= MAX_JPEG_BYTES &&
            frame->buf[0] == 0xff && frame->buf[1] == 0xd8 &&
            frame->buf[frame->len - 2] == 0xff && frame->buf[frame->len - 1] == 0xd9) {
            publish_frame(frame);
        } else if (now - last_error_us >= STATUS_INTERVAL_US) {
            publish_status("capture_failed");
            last_error_us = now;
        }
        if (frame) {
            esp_camera_fb_return(frame);
        }
        if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRAME_PERIOD_MS)) == pdFALSE) {
            last_wake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void bench_camera_start(void)
{
    if (xTaskCreate(camera_task, TAG, 4096, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Unable to start camera capture");
    }
}
