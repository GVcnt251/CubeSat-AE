#include "wheel.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "can_port.h"
#include "moteus_proto.h"

static const char *TAG = "wheel";

#define TWO_PI 6.28318530718f
#define WHEEL_REPLY_TIMEOUT_US 50000ULL
#define WHEEL_RX_LIMIT 64U

static bool command_valid(const cc_out_t *command)
{
    return command != NULL &&
           isfinite(command->wheel_cmd_rps) &&
           isfinite(command->torque_max_nm) &&
           isfinite(command->accel_limit_rps2) &&
           command->torque_max_nm > 0.0f &&
           command->accel_limit_rps2 > 0.0f;
}

#ifdef CONFIG_CUBE_WHEEL_FAKE

static float s_velocity[WHEEL_AXIS_COUNT];
static uint64_t s_last_us;
static bool s_have_time;

int wheel_init(void)
{
    memset(s_velocity, 0, sizeof(s_velocity));
    s_last_us = 0;
    s_have_time = false;

    ESP_LOGW(TAG, "Three simulated wheels enabled");
    return 0;
}

void wheels_update(
    const cc_out_t commands[WHEEL_AXIS_COUNT],
    wheel_state_t states[WHEEL_AXIS_COUNT])
{
    if (states == NULL) {
        return;
    }

    uint64_t now = (uint64_t)esp_timer_get_time();
    float dt = s_have_time
        ? (float)(now - s_last_us) * 1e-6f
        : 0.005f;

    s_last_us = now;
    s_have_time = true;

    for (unsigned axis = 0; axis < WHEEL_AXIS_COUNT; ++axis) {
        const cc_out_t *command =
            commands != NULL ? &commands[axis] : NULL;

        if (command_valid(command) && command->enable) {
            float step = command->accel_limit_rps2 * dt;
            float delta =
                command->wheel_cmd_rps - s_velocity[axis];

            s_velocity[axis] +=
                fmaxf(-step, fminf(step, delta));
        }

        /*
         * Disabled wheels coast at their current simulated velocity.
         * This is a simple model, without friction or motor dynamics.
         */
        states[axis] = (wheel_state_t) {
            .vel_rps = s_velocity[axis],
            .torque_nm = 0.0f,
            .vbat_v = 22.2f,
            .valid = true
        };
    }
}

#else

typedef struct {
    wheel_state_t state;
    uint64_t last_reply_us;
    bool have_reply;
    bool motor_fault;
} wheel_runtime_t;

static wheel_runtime_t s_wheels[WHEEL_AXIS_COUNT];
static bool s_can_ok;

static const unsigned s_motor_id[WHEEL_AXIS_COUNT] = {
    CONFIG_CUBE_MOTEUS_ID_X,
    CONFIG_CUBE_MOTEUS_ID_Y,
    CONFIG_CUBE_MOTEUS_ID
};

static const int s_sign[WHEEL_AXIS_COUNT] = {
    CONFIG_CUBE_WHEEL_SIGN_X,
    CONFIG_CUBE_WHEEL_SIGN_Y,
    CONFIG_CUBE_WHEEL_SIGN
};

static const char *const s_axis_name[WHEEL_AXIS_COUNT] = {
    "X", "Y", "Z"
};

int wheel_init(void)
{
    memset(s_wheels, 0, sizeof(s_wheels));
    s_can_ok = false;

    unsigned configured = 0;

    for (unsigned axis = 0; axis < WHEEL_AXIS_COUNT; ++axis) {
        if (s_motor_id[axis] == 0) {
            ESP_LOGW(TAG, "Wheel %s is not configured",
                     s_axis_name[axis]);
            continue;
        }

        if (s_motor_id[axis] > 127 ||
            (s_sign[axis] != 1 && s_sign[axis] != -1)) {
            ESP_LOGE(TAG, "Invalid configuration for wheel %s",
                     s_axis_name[axis]);
            return -1;
        }

        for (unsigned other = 0; other < axis; ++other) {
            if (s_motor_id[axis] == s_motor_id[other]) {
                ESP_LOGE(TAG,
                         "Duplicate CAN ID for wheels %s and %s",
                         s_axis_name[other], s_axis_name[axis]);
                return -1;
            }
        }

        configured++;
    }

    if (configured == 0) {
        ESP_LOGE(TAG, "No wheels configured");
        return -1;
    }

    s_can_ok = can_port_init() == ESP_OK;
    return s_can_ok ? 0 : -1;
}

static void wheel_apply_reply(
    unsigned axis,
    const moteus_status_t *status)
{
    wheel_runtime_t *wheel = &s_wheels[axis];

    /*
     * A reported fault blocks actuation immediately, even when
     * the reply does not contain all requested measurements.
     */
    if (status->have_aux && status->fault != 0) {
        if (!wheel->motor_fault) {
            ESP_LOGE(TAG, "Wheel %s fault: %u",
                     s_axis_name[axis],
                     (unsigned)status->fault);
        }

        wheel->motor_fault = true;
        wheel->state.valid = false;
    }

    /*
     * Only complete replies refresh feedback and clear a previous
     * fault. Partial replies cannot make an old state usable.
     */
    if (!status->have_mode ||
        !status->have_motion ||
        !status->have_aux) {
        return;
    }

    if (!isfinite(status->velocity_rev_s) ||
        !isfinite(status->torque_nm) ||
        !isfinite(status->voltage_v)) {
        wheel->have_reply = false;
        wheel->state.valid = false;
        return;
    }

    float velocity =
        (float)s_sign[axis] * status->velocity_rev_s * TWO_PI;

    if (!isfinite(velocity)) {
        wheel->have_reply = false;
        wheel->state.valid = false;
        return;
    }

    wheel->state.vel_rps = velocity;
    wheel->state.torque_nm =
        (float)s_sign[axis] * status->torque_nm;
    wheel->state.vbat_v = status->voltage_v;

    wheel->motor_fault = status->fault != 0;
    wheel->last_reply_us = (uint64_t)esp_timer_get_time();
    wheel->have_reply = true;
}

static void wheels_receive(void)
{
    can_frame_t frame;

    /*
     * Bound work per call so a busy CAN bus cannot keep this
     * control task inside the receive loop indefinitely.
     */
    for (unsigned received = 0;
         received < WHEEL_RX_LIMIT;
         ++received) {
        if (!can_port_recv(&frame)) {
            break;
        }

        if (frame.len > sizeof(frame.data)) {
            continue;
        }

        for (unsigned axis = 0;
             axis < WHEEL_AXIS_COUNT;
             ++axis) {
            if (s_motor_id[axis] == 0) {
                continue;
            }

            if (frame.id !=
                moteus_can_id(s_motor_id[axis], 0, 0)) {
                continue;
            }

            moteus_status_t status = { 0 };

            if (moteus_parse_reply(
                    frame.data, frame.len, &status) == 0) {
                wheel_apply_reply(axis, &status);
            }

            break;
        }
    }
}

static void wheel_send(
    unsigned axis,
    const cc_out_t *command)
{
    wheel_runtime_t *wheel = &s_wheels[axis];
    uint8_t buffer[64];
    size_t length;

    bool enabled = command_valid(command) &&
                   command->enable &&
                   wheel->state.valid;

    if (enabled) {
        length = moteus_pack_velocity(
            buffer,
            (float)s_sign[axis] *
                command->wheel_cmd_rps / TWO_PI,
            command->torque_max_nm,
            command->accel_limit_rps2 / TWO_PI,
            1);
    } else {
        /* STOP also requests feedback, allowing initial discovery. */
        length = moteus_pack_stop(buffer, 1);
    }

    can_frame_t frame = {
        .id = moteus_can_id(0, s_motor_id[axis], 1),
        .len = 0
    };

    if (length == 0 || length > sizeof(frame.data)) {
        wheel->state.valid = false;
        wheel->have_reply = false;
        return;
    }

    frame.len = (uint8_t)length;
    memcpy(frame.data, buffer, length);

    if (can_port_send(&frame) != ESP_OK) {
        wheel->state.valid = false;
        wheel->have_reply = false;
    }
}

void wheels_update(
    const cc_out_t commands[WHEEL_AXIS_COUNT],
    wheel_state_t states[WHEEL_AXIS_COUNT])
{
    if (states == NULL) {
        return;
    }

    if (!s_can_ok) {
        for (unsigned axis = 0;
             axis < WHEEL_AXIS_COUNT;
             ++axis) {
            s_wheels[axis].state.valid = false;
            states[axis] = s_wheels[axis].state;
        }
        return;
    }

    /* Receive once and route replies to their corresponding wheel. */
    wheels_receive();

    uint64_t now = (uint64_t)esp_timer_get_time();

    for (unsigned axis = 0; axis < WHEEL_AXIS_COUNT; ++axis) {
        wheel_runtime_t *wheel = &s_wheels[axis];

        wheel->state.valid =
            s_motor_id[axis] != 0 &&
            wheel->have_reply &&
            !wheel->motor_fault &&
            (now - wheel->last_reply_us <
                WHEEL_REPLY_TIMEOUT_US);
    }

    for (unsigned axis = 0; axis < WHEEL_AXIS_COUNT; ++axis) {
        if (s_motor_id[axis] != 0) {
            const cc_out_t *command =
                commands != NULL ? &commands[axis] : NULL;

            wheel_send(axis, command);
        }

        states[axis] = s_wheels[axis].state;
    }
}

#endif

wheel_state_t wheel_update(const cc_out_t *command)
{
    cc_out_t commands[WHEEL_AXIS_COUNT] = { 0 };
    wheel_state_t states[WHEEL_AXIS_COUNT] = { 0 };

    if (command != NULL) {
        commands[WHEEL_AXIS_Z] = *command;
    }

    wheels_update(commands, states);
    return states[WHEEL_AXIS_Z];
}