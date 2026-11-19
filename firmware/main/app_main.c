/*
 * app_main.c - liga o nÃºcleo de controlo (core/) ao hardware.
 *
 *   control_task  200 Hz, nÃºcleo 1, prioridade 20 : IMU -> cc3_step_with_imu -> roda (moteus)
 *   camera_task   ~20 fps, nÃºcleo 0               : cÃ¢mara -> cc_detect -> cc3_on_detection
 *   comms_task    200 Hz, nÃºcleo 0                : UDP (TC in, TM out)
 *
 * Nada de lÃ³gica de controlo aqui: estÃ¡ toda em core/, igual ao simulador (SIL).
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

cc3_t g_core;
SemaphoreHandle_t g_core_mtx;

static bool s_imu_ok;

/* DisponibilizaÃ§Ã£o dos mÃ³dulos necessÃ¡rios, etiqueta de diagnÃ³stico 
preparaÃ§Ã£o do estado partilhado de controlo, proteÃ§Ã£o por mutex e indicador
da inicializaÃ§Ã£o da IMU*/

static void control_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
    uint32_t n = 0, late = 0, over_budget = 0;
    uint64_t max_cycle_us = 0;
    const uint64_t budget_us = 1000000ULL / CONTROL_HZ;

    #ifndef CONFIG_CUBE_WHEEL_FAKE
        uint32_t stale = 0;
    #endif
    float vbat = 0;
    cc3_out_t out = { 0 };
    imu_sample_t s = { 0 };
    wheel_state_t w[WHEEL_AXIS_COUNT] = { 0 };
    
    /*PreparaÃ§Ã£o para funcionamento a 200HZ (Control_hz), guarda separadamente
    mediÃ§Ãµes e comandos e verifica atrasos e/ou ausÃªncia da resposta da wheel*/

    for (;;) {
        if (xTaskDelayUntil(&last, period) == pdFALSE) late++;      /* perdeu o prazo */
        uint64_t t = (uint64_t)esp_timer_get_time();

        bool imu_sample_ok = false;

        if (s_imu_ok) {
            imu_sample_ok = imu_read(&s) == ESP_OK;
        }

        float mag[3] = { 0 };
        bool have_mag = imu_sample_ok &&
            (n % MAG_DIV == 0) && mag_read(mag) == ESP_OK;
        if (n % 40 == 0) vbat = battery_read_v();

        cc3_meas_t m = { 0 };
        m.vbat_v = vbat > 0.0f ? vbat : 0.0f;
        m.mag_valid = have_mag;
        if (have_mag) {
            memcpy(m.mag, mag, sizeof(m.mag));
        }

        for (unsigned axis = 0; axis < WHEEL_AXIS_COUNT; ++axis) {
            m.wheel_rps[axis] = w[axis].vel_rps;
            m.wheel_torque[axis] = w[axis].torque_nm;
            if (w[axis].valid) {
                m.motor_valid_mask |= (uint8_t)(1u << axis);
                if (m.vbat_v <= 0.0f && w[axis].vbat_v > 0.0f) {
                    m.vbat_v = w[axis].vbat_v;
                }
            }
        }
        memcpy(m.gyro, s.gyro, sizeof(m.gyro));
        memcpy(m.accel, s.accel, sizeof(m.accel));

        CORE_LOCK();
        /*a tarefa espera pelo instante previsto, tenta ler os sensores Ã s respetivas 
        frequÃªncias, reÃºne as mediÃ§Ãµes e sÃ³ depois obtÃ©m acesso exclusivo ao nÃºcleo. 
        AtÃ© aqui, ainda nÃ£o calculou nem enviou um novo comando Ã  roda.*/
        
        cc_imu_status_t imu_status = cc3_step_with_imu(
            &g_core, t, &m, imu_sample_ok, &out);

        cc_mode_t mode = g_core.mode;

        CORE_UNLOCK();

        /* Sem IMU utilizÃ¡vel, bloquear modos dependentes da orientaÃ§Ã£o. */
        if (imu_status != CC_IMU_OK && mode != CC_MODE_WHEEL) {
            for (unsigned axis = 0; axis < WHEEL_AXIS_COUNT; ++axis) {
                out.axis[axis].enable = 0;
            }
        }
        wheels_update(out.axis, w);                                     /* envia jÃ¡ o comando; o estado serve o ciclo seguinte */
#ifndef CONFIG_CUBE_WHEEL_FAKE
        /* All motors are required in the attitude-dependent modes.
         * WHEEL keeps independent operation of each available motor.
         * The core and wheel driver already guard actuation each cycle.
         */
        bool all_motors_valid = true;
        for (unsigned axis = 0; axis < WHEEL_AXIS_COUNT; ++axis) {
            all_motors_valid = all_motors_valid && w[axis].valid;
        }
        bool needs_all_motors = mode == CC_MODE_RATE ||
            mode == CC_MODE_HEADING || mode == CC_MODE_TRACK;

        if (needs_all_motors && !all_motors_valid) {
            if (++stale == 40) { /* 40 cycles: nominally 200 ms at 200 Hz. */
                cc_cmd_t idle = { .type = CC_CMD_MODE, .a = CC_MODE_IDLE };
                CORE_LOCK();
                int applied = cc3_command_legacy(&g_core, &idle);
                CORE_UNLOCK();
                if (applied) {
                    ESP_LOGE(TAG, "Motor feedback unavailable: mode IDLE");
                }
            }
        } else {
            stale = 0;
        }
/*o nÃºcleo calcula, o firmware aplica a proteÃ§Ã£o da IMU, o mÃ³dulo da roda 
comunica com o hardware e a ausÃªncia consecutiva de respostas provoca um pedido 
interno de IDLE. A leitura do cÃ³digo demonstra essa sequÃªncia; 
a paragem efetiva da roda exige validaÃ§Ã£o no sistema fÃsico.*/

#endif
        uint64_t cycle_us = (uint64_t)esp_timer_get_time() - t;

        if (cycle_us > max_cycle_us) {
            max_cycle_us = cycle_us;
        }

        if (cycle_us > budget_us) {
            over_budget++;
        }

        if (++n % (CONTROL_HZ * 10) == 0) {
            ESP_LOGI(TAG,
                     "Controlo: max=%llu us, acima do periodo=%u, atrasos=%u",
                     (unsigned long long)max_cycle_us,
                     (unsigned)over_budget,
                     (unsigned)late);

            max_cycle_us = 0;
            over_budget = 0;
            late = 0;
        }
    }
}

/*o firmware mede a duraÃ§Ã£o do trabalho, guarda o mÃ¡ximo e conta ultrapassagens 
e atrasos. Esses registos ajudam a verificar o cumprimento dos 200 Hz nas 
condiÃ§Ãµes ensaiadas, mas nÃ£o demonstram, por si sÃ³, que o controlo 
fÃsico do CubeSat funciona corretamente.*/

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
        ESP_LOGE(TAG, "Falha ao criar mutex do nÃºcleo");
        return;
    }

    if (!cc3_init(&g_core, NULL)) {
        ESP_LOGE(TAG, "Failed to initialize the three-axis core");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    const float calib_s = g_core.cfg.axis[WHEEL_AXIS_Z].calib_s;
    if (battery_init() != 0) {
    ESP_LOGE(TAG, "Falha na inicializaÃ§Ã£o da leitura da bateria");
    }
    s_imu_ok = imu_init() == ESP_OK;
    if (!s_imu_ok) {
        ESP_LOGE(TAG,
                "Falha na inicializaÃ§Ã£o da IMU: corrigir a causa e reiniciar o ESP32. "
                "WHEEL requer calibraÃ§Ã£o inicial concluÃda.");
    }

    /*Se NVS permanecer indisponÃvel, provoca a interrupÃ§Ã£o fatal da execuÃ§Ã£o normal, 
    se houver uma falha no mutex hÃ¡ o registo do erro e a saÃda da app_main, 
    se houver um erro da inicializaÃ§Ã£o da bateria, hÃ¡ o registo do erro e a 
    continuaÃ§Ã£o do programa e se houver uma falha na inicializaÃ§Ã£o da IMU, 
    hÃ¡ o registo do erro, o indicaÃ§Ã£o de leitura a falso, mas a continuaÃ§Ã£o.*/

    if (wheel_init() != 0) ESP_LOGE(TAG, "roda indisponÃvel (driver CAN)");
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
            ESP_LOGE(TAG, "Falha ao criar tarefa de comunicaÃ§Ãµes");
            ESP_ERROR_CHECK(ESP_FAIL);
        }

        task_result = xTaskCreatePinnedToCore(
            camera_task, "camera", 8192, NULL, 6, NULL, 0);
        if (task_result != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar tarefa da cÃ¢mara");
            ESP_ERROR_CHECK(ESP_FAIL);
        }
    ESP_LOGI(TAG, "cubo a arrancar: calibraÃ§Ã£o do giroscÃ³pio nos primeiros %.1f s (cubo PARADO)", calib_s);
}
/*20, 5 e 6 Ã© a prioridade. Prioridade maior significa preferÃªncia de 
execuÃ§Ã£o quando existem tarefas prontas a executar no mesmo nÃºcleo
(nÃ£o Ã© uma percentagem fixa do processador).

O app_main.c inicializa os mÃ³dulos do CubeSat e cria as tarefas de controlo, 
cÃ¢mara e comunicaÃ§Ãµes. A tarefa de controlo funciona nominalmente a 200 Hz, 
reÃºne mediÃ§Ãµes, chama o nÃºcleo de controlo e envia os comandos Ã  roda.
Protege o acesso ao nÃºcleo com um mutex, aplica proteÃ§Ãµes 
perante falhas e regista os tempos de execuÃ§Ã£o e atrasos.*/
