#pragma once

#include <stdbool.h>
#include "cube_core.h"

enum {
    WHEEL_AXIS_X = 0,
    WHEEL_AXIS_Y,
    WHEEL_AXIS_Z,
    WHEEL_AXIS_COUNT
};

typedef struct {
    float vel_rps;     /* Angular velocity in rad/s, in core convention. */
    float torque_nm;
    float vbat_v;
    bool valid;        /* Recent, complete feedback without motor fault. */
} wheel_state_t;

int wheel_init(void);

/* Update all wheels together. Commands and states use XYZ order. */
void wheels_update(
    const cc_out_t commands[WHEEL_AXIS_COUNT],
    wheel_state_t states[WHEEL_AXIS_COUNT]);

/* Compatibility with the existing single-axis firmware. */
wheel_state_t wheel_update(const cc_out_t *command);