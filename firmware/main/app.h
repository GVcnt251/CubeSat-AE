#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cube_core3.h"

/* Shared by control, camera and communications.
 * Every access after task creation must hold this mutex.
 */
extern cc3_t g_core;
extern SemaphoreHandle_t g_core_mtx;

/* Telemetry configuration (TC_STREAM_CFG). */
extern volatile uint8_t g_state_hz, g_vision_hz, g_img_every;

#define CORE_LOCK()   xSemaphoreTake(g_core_mtx, portMAX_DELAY)
#define CORE_UNLOCK() xSemaphoreGive(g_core_mtx)
