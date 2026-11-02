/*
 * app_main.c - liga o núcleo de controlo (core/) ao hardware.
 *
 *   control_task  200 Hz, núcleo 1, prioridade 20 : IMU -> cc_step -> roda (moteus)
 *   camera_task   ~20 fps, núcleo 0               : câmara -> cc_detect -> cc_on_detection
 *   comms_task    200 Hz, núcleo 0                : UDP (TC in, TM out)
 *
 * Nada de lógica de controlo aqui: está toda em core/, igual ao simulador (SIL).
 */
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "app.h"
#include "battery.h"
#include "board.h"
#include "camera.h"
#include "comms.h"
#include "imu.h"
#include "wheel.h"
#include "esp_err.h"

static const char *TAG = "cube";

cc_t g_core;
SemaphoreHandle_t g_core_mtx;

static bool s_imu_ok;

static void control_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
    uint32_t n = 0, late = 0;

    #ifndef CONFIG_CUBE_WHEEL_FAKE
        uint32_t stale = 0;
    #endif
    float vbat = 0;
    cc_out_t out = { 0 };
    imu_sample_t s = { 0 };
    wheel_state_t w = { 0 };

    for (;;) {
        if (xTaskDelayUntil(&last, period) == pdFALSE) late++;      /* perdeu o prazo */
        uint64_t t = (uint64_t)esp_timer_get_time();

        bool imu_sample_ok = false;

        if (s_imu_ok) {
            imu_sample_ok = imu_read(&s) == ESP_OK;
        }

        float mag[3];
        bool have_mag = (n % MAG_DIV == 0) && mag_read(mag) == ESP_OK;
        if (n % 40 == 0) vbat = battery_read_v();

        cc_meas_t m = { .wheel_rps = w.vel_rps, .wheel_torque = w.torque_nm, .vbat_v = vbat > 0 ? vbat : w.vbat_v };
        memcpy(m.gyro, s.gyro, sizeof(m.gyro));
        memcpy(m.accel, s.accel, sizeof(m.accel));

        CORE_LOCK();

        cc_imu_status_t imu_status = cc_step_with_imu(
            &g_core, t, &m, imu_sample_ok, &out);

        cc_mode_t mode = g_core.mode;

        if (have_mag && imu_sample_ok &&
            (imu_status == CC_IMU_OK || mode == CC_MODE_CALIB)) {
            cc_on_mag(&g_core, t, mag);
        }

        CORE_UNLOCK();

        /* Sem IMU utilizável, bloquear modos dependentes da orientação. */
        if (imu_status != CC_IMU_OK && mode != CC_MODE_WHEEL) {
            out.enable = 0;
        }
        w = wheel_update(&out);                                     /* envia já o comando; o estado serve o ciclo seguinte */
#ifndef CONFIG_CUBE_WHEEL_FAKE
        if (!w.valid) {
            if (++stale == 40) {                                    /* 200 ms sem resposta do moteus */
                cc_cmd_t idle = { .type = CC_CMD_MODE, .a = CC_MODE_IDLE };
                CORE_LOCK(); cc_command(&g_core, &idle); CORE_UNLOCK();
                ESP_LOGE(TAG, "moteus sem resposta: modo IDLE");
            }
        } else stale = 0;
#endif
        if (++n % (CONTROL_HZ * 10) == 0 && late) { ESP_LOGW(TAG, "%u ciclos de controlo atrasados", (unsigned)late); late = 0; }
    }
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();

    if (e == ESP_ERR_NVS_NO_FREE_PAGES ||
        e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }

ESP_ERROR_CHECK(e);

    g_core_mtx = xSemaphoreCreateMutex();

    if (g_core_mtx == NULL) {
        ESP_LOGE(TAG, "Falha ao criar mutex do núcleo");
        return;
    }

cc_init(&g_core, NULL);
    if (battery_init() != 0) {
    ESP_LOGE(TAG, "Falha na inicialização da leitura da bateria");
    }
    s_imu_ok = imu_init() == ESP_OK;
    if (!s_imu_ok) {
        ESP_LOGE(TAG,
                "Falha na inicialização da IMU: corrigir a causa e reiniciar o ESP32. "
                "WHEEL requer calibração inicial concluída.");
    }
    if (wheel_init() != 0) ESP_LOGE(TAG, "roda indisponível (driver CAN)");
    comms_init();

        BaseType_t task_result;

        task_result = xTaskCreatePinnedToCore(
            control_task, "control", 4096, NULL, 20, NULL, 1);
        if (task_result != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar tarefa de controlo");
            ESP_ERROR_CHECK(ESP_FAIL);
        }

        task_result = xTaskCreatePinnedToCore(
            comms_task, "comms", 6144, NULL, 5, NULL, 0);
        if (task_result != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar tarefa de comunicações");
            ESP_ERROR_CHECK(ESP_FAIL);
        }

        task_result = xTaskCreatePinnedToCore(
            camera_task, "camera", 8192, NULL, 6, NULL, 0);
        if (task_result != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar tarefa da câmara");
            ESP_ERROR_CHECK(ESP_FAIL);
        }
    ESP_LOGI(TAG, "cubo a arrancar: calibração do giroscópio nos primeiros %.1f s (cubo PARADO)", g_core.cfg.calib_s);
}
