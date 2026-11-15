#include "wheel.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "can_port.h"
#include "moteus_proto.h"

static const char *TAG = "wheel";
#define TWO_PI 6.28318530718f

#ifdef CONFIG_CUBE_WHEEL_FAKE

/* Roda simulada: seguimento com limite de aceleração (sem moteus). */
static float s_v;
static uint64_t s_last;

int wheel_init(void)
{
    ESP_LOGW(TAG, "roda SIMULADA em software (CONFIG_CUBE_WHEEL_FAKE)");
    return 0;
}

wheel_state_t wheel_update(const cc_out_t *c)
{
    uint64_t now = (uint64_t)esp_timer_get_time();
    float dt = s_last ? (float)(now - s_last) * 1e-6f : 0.005f;
    s_last = now;

    if (c->enable) {
        float step = c->accel_limit_rps2 * dt;
        float d = c->wheel_cmd_rps - s_v;
        s_v += d > step ? step : (d < -step ? -step : d);
    }

    wheel_state_t w = {
        .vel_rps = s_v,
        .torque_nm = 0,
        .vbat_v = 22.2f,
        .valid = true,
    };
    return w;
}

#else

/* Moteus por CAN-FD. */
typedef struct {
    wheel_state_t state;
    uint64_t last_reply_us;
    bool have_reply;
    bool motor_fault;
} wheel_runtime_t;

enum {
    WHEEL_AXIS_X = 0,
    WHEEL_AXIS_Y,
    WHEEL_AXIS_Z,
    WHEEL_AXIS_COUNT
};

static wheel_runtime_t s_wheels[WHEEL_AXIS_COUNT];
static bool s_can_ok;

int wheel_init(void)
{
    memset(s_wheels, 0, sizeof(s_wheels));

    s_can_ok = can_port_init() == ESP_OK;
    return s_can_ok ? 0 : -1;
}

static void wheel_apply_reply(wheel_runtime_t *wheel,
                              const moteus_status_t *status,
                              float sign)
{
    if (status->have_motion) {
        wheel->state.vel_rps = sign * status->velocity_rev_s * TWO_PI;
        wheel->state.torque_nm = sign * status->torque_nm;
    }

    if (status->have_aux) {
        wheel->state.vbat_v = status->voltage_v;

        bool fault = status->fault != 0;
        if (fault && !wheel->motor_fault) {
            ESP_LOGE(TAG, "moteus em falha: código %u", status->fault);
        }
        wheel->motor_fault = fault;
    }

    if (status->have_mode && status->have_motion && status->have_aux) {
        wheel->last_reply_us = (uint64_t)esp_timer_get_time();
        wheel->have_reply = true;
    }
}

wheel_state_t wheel_update(const cc_out_t *c)
{
    wheel_runtime_t *const wheel = &s_wheels[WHEEL_AXIS_Z];

    if (!s_can_ok) {
        wheel->state.valid = false;
        return wheel->state;
    }

    const float sign =
        (CONFIG_CUBE_WHEEL_SIGN < 0) ? -1.0f : 1.0f;

    /* Receber e verificar o estado antes de escolher o comando. */
    can_frame_t r;
    while (can_port_recv(&r)) {
        if (r.id != moteus_can_id(CONFIG_CUBE_MOTEUS_ID, 0, 0)) {
            continue;
        }

        moteus_status_t st;
        if (moteus_parse_reply(r.data, r.len, &st) != 0) {
            continue;
        }

        wheel_apply_reply(wheel, &st, sign);
    }

    wheel->state.valid = wheel->have_reply && !wheel->motor_fault &&
        ((uint64_t)esp_timer_get_time() - wheel->last_reply_us < 50000);

    /* Sem estado válido, enviar STOP com pedido de resposta. */
    uint8_t buf[64];
    size_t n;

    if (c->enable && wheel->state.valid) {
        n = moteus_pack_velocity(
            buf,
            sign * c->wheel_cmd_rps / TWO_PI,
            c->torque_max_nm,
            c->accel_limit_rps2 / TWO_PI,
            1);
    } else {
        n = moteus_pack_stop(buf, 1);
    }

    can_frame_t f = {
        .id = moteus_can_id(0, CONFIG_CUBE_MOTEUS_ID, 1),
        .len = (uint8_t)n,
    };
    memcpy(f.data, buf, n);

    if (can_port_send(&f) != ESP_OK) {
        wheel->state.valid = false;
        wheel->have_reply = false;
    }

    return wheel->state;
}

#endif