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
/* ---- roda simulada: seguimento com limite de aceleração (sem moteus) */
static float s_v;
static uint64_t s_last;

int wheel_init(void) { ESP_LOGW(TAG, "roda SIMULADA em software (CONFIG_CUBE_WHEEL_FAKE)"); return 0; }

wheel_state_t wheel_update(const cc_out_t *c)
{
    uint64_t now = (uint64_t)esp_timer_get_time();
    float dt = s_last ? (float)(now - s_last) * 1e-6f : 0.005f;
    s_last = now;
    if (c->enable) {
        float step = c->accel_limit_rps2 * dt, d = c->wheel_cmd_rps - s_v;
        s_v += d > step ? step : (d < -step ? -step : d);
    }
    wheel_state_t w = { .vel_rps = s_v, .torque_nm = 0, .vbat_v = 22.2f, .valid = true };
    return w;
}
#else
/* ---- moteus por CAN-FD */
static wheel_state_t s_state;
static uint64_t s_last_reply_us;
static bool s_have_reply;
static bool s_motor_fault;
static bool s_can_ok;

int wheel_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_last_reply_us = 0;
    s_have_reply = false;
    s_motor_fault = false;

    s_can_ok = can_port_init() == ESP_OK;
    return s_can_ok ? 0 : -1;
}

wheel_state_t wheel_update(const cc_out_t *c)
{
    if (!s_can_ok) {
        s_state.valid = false;
        return s_state;
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

        if (st.have_motion) {
            s_state.vel_rps = sign * st.velocity_rev_s * TWO_PI;
            s_state.torque_nm = sign * st.torque_nm;
        }

        if (st.have_aux) {
            s_state.vbat_v = st.voltage_v;

            bool fault = st.fault != 0;
            if (fault && !s_motor_fault) {
                ESP_LOGE(TAG, "moteus em falha: código %u", st.fault);
            }
            s_motor_fault = fault;
        }

        if (st.have_mode && st.have_motion && st.have_aux) {
            s_last_reply_us = (uint64_t)esp_timer_get_time();
            s_have_reply = true;
        }
    }

    s_state.valid = s_have_reply && !s_motor_fault &&
        ((uint64_t)esp_timer_get_time() - s_last_reply_us < 50000);

    /* Sem estado válido, enviar STOP com pedido de resposta. */
    uint8_t buf[64];
    size_t n;

    if (c->enable && s_state.valid) {
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
        s_state.valid = false;
        s_have_reply = false;
    }

    return s_state;
}
#endif
