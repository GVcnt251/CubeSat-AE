#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cube_core.h"

/* O núcleo é partilhado por 3 tarefas (controlo, câmara, comunicações): TODO o acesso passa por este mutex.
 * Só cc_step / cc_on_mag / cc_on_detection / cc_command / cc_get_tlm lhe tocam, e todas são rápidas (µs). */
extern cc_t g_core;
extern SemaphoreHandle_t g_core_mtx;

/* configuração de telemetria (TC_STREAM_CFG) */
extern volatile uint8_t g_state_hz, g_vision_hz, g_img_every;

#define CORE_LOCK()   xSemaphoreTake(g_core_mtx, portMAX_DELAY)
#define CORE_UNLOCK() xSemaphoreGive(g_core_mtx)
