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

int wheel_init(void)
{
    if (can_port_init() != ESP_OK) return -1;
    return 0;
}

wheel_state_t wheel_update(const cc_out_t *c)
{
    uint8_t buf[64];
    const float sign = (CONFIG_CUBE_WHEEL_SIGN < 0) ? -1.0f : 1.0f;
    size_t n = c->enable
        ? moteus_pack_velocity(buf, sign * c->wheel_cmd_rps / TWO_PI, c->torque_max_nm, c->accel_limit_rps2 / TWO_PI, 1)
        : moteus_pack_stop(buf, 1);
    can_frame_t f = { .id = moteus_can_id(0, CONFIG_CUBE_MOTEUS_ID, 1), .len = (uint8_t)n };
    memcpy(f.data, buf, n);
    can_port_send(&f);

    can_frame_t r;
    while (can_port_recv(&r)) {
        if (r.id != moteus_can_id(CONFIG_CUBE_MOTEUS_ID, 0, 0)) continue;
        moteus_status_t st;
        if (moteus_parse_reply(r.data, r.len, &st) != 0) continue;
        if (st.have_motion) { s_state.vel_rps = sign * st.velocity_rev_s * TWO_PI; s_state.torque_nm = sign * st.torque_nm; }
        if (st.have_aux)    s_state.vbat_v = st.voltage_v;
        if (st.have_mode && st.fault) ESP_LOGE(TAG, "moteus em falha: código %u", st.fault);
        s_last_reply_us = (uint64_t)esp_timer_get_time();
    }
    s_state.valid = (uint64_t)esp_timer_get_time() - s_last_reply_us < 50000;
    return s_state;
}
#endif
