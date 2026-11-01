#include "cube_core.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>

#define PI_F 3.14159265358979f
#define US(x) ((double)(x) * 1e-6)

static float wrapf(float a)
{
    a = remainderf(a, 2.0f * PI_F);
    return a;
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ------------------------------------------------------------------ config */

void cc_config_default(cc_config_t *g)
{
    memset(g, 0, sizeof(*g));
    /* câmara: 320x240, HFOV ~65.5 deg (do slide: 83.9 px <-> 18.6 deg => f = 249 px) */
    g->cam_fx = 249.0f; g->cam_fy = 249.0f; g->cam_cx = 159.5f; g->cam_cy = 119.5f;
    cc_det_cfg_default(&g->det);
    g->calib_s = 1.5f;

    g->q_psi    = 1e-7f;
    g->q_bias   = 1e-9f;
    g->q_target = 1e-4f;
    g->r_mag    = 0.05f * 0.05f;          /* ~2.9 deg */
    g->r_cam    = 0.007f * 0.007f;        /* ~0.4 deg */
    g->cam_gate = 0.30f;
    g->mag_nom_uT = 44.0f; g->mag_tol = 0.40f;
    g->use_mag  = 1;

    g->k_theta   = 3.0f;
    g->rate_kp   = 5.0f;
    g->rate_ki   = 300.0f;
    g->w_ref_max = 0.70f;                 /* 40 deg/s */
    g->a_ref_max = 4.0f;
    g->wheel_lim = 314.16f;               /* 3000 rpm */
    g->torque_max = 0.20f;
    g->wheel_accel = 800.0f;

    g->t_lock = 0.30f; g->t_search = 5.0f; g->search_rate = 0.35f;
}

void cc_init(cc_t *c, const cc_config_t *cfg)
{
    memset(c, 0, sizeof(*c));
    if (cfg) c->cfg = *cfg; else cc_config_default(&c->cfg);
    c->mode = CC_MODE_CALIB;
    c->search_dir = 1.0f;
}

/* ---------------------------------------------------------------- histórico */

static void hist_push(cc_t *c, uint64_t t)
{
    c->t_hist[c->h_head] = t;
    c->psi_hist[c->h_head] = c->x[0];
    c->e_hist[c->h_head] = c->x[2] - c->x[0];
    c->h_head = (c->h_head + 1) % CC_HIST;
    if (c->h_count < CC_HIST) c->h_count++;
}

/* devolve psi e e no instante t (interpolação linear; satura nos extremos) */
static void hist_get(const cc_t *c, uint64_t t, float *psi, float *e)
{
    if (c->h_count == 0) { *psi = c->x[0]; *e = c->x[2] - c->x[0]; return; }
    int newest = (c->h_head - 1 + CC_HIST) % CC_HIST;
    if (t >= c->t_hist[newest]) { *psi = c->psi_hist[newest]; *e = c->e_hist[newest]; return; }
    int idx = newest;
    for (int k = 1; k < c->h_count; k++) {
        int prev = (newest - k + CC_HIST) % CC_HIST;
        if (c->t_hist[prev] <= t) {
            double span = (double)(c->t_hist[idx] - c->t_hist[prev]);
            float a = span > 0 ? (float)((double)(t - c->t_hist[prev]) / span) : 0.0f;
            *psi = c->psi_hist[prev] + a * (c->psi_hist[idx] - c->psi_hist[prev]);
            *e   = c->e_hist[prev]   + a * (c->e_hist[idx]   - c->e_hist[prev]);
            return;
        }
        idx = prev;
    }
    *psi = c->psi_hist[idx]; *e = c->e_hist[idx];   /* mais antigo que o histórico */
}

/* A correcção aplicada agora é propagada retroactivamente, para que a medida seguinte
 * (capturada antes desta correcção chegar) não a conte duas vezes. */
static void hist_shift(cc_t *c, float dpsi, float de)
{
    for (int k = 0; k < c->h_count; k++) { c->psi_hist[k] += dpsi; c->e_hist[k] += de; }
}

/* ---------------------------------------------------------------- estimador */
/* x = [psi, bias, psi_alvo];  psi' = gyro - bias;  bias' = 0;  psi_alvo' = 0 */

static void kf_predict(cc_t *c, float dt, float gyro)
{
    float (*P)[3] = c->P;
    c->x[0] += (gyro - c->x[1]) * dt;

    float p00 = P[0][0] - 2.0f * dt * P[0][1] + dt * dt * P[1][1] + c->cfg.q_psi * dt;
    float p01 = P[0][1] - dt * P[1][1];
    float p02 = P[0][2] - dt * P[1][2];
    P[0][0] = p00; P[0][1] = p01; P[1][0] = p01; P[0][2] = p02; P[2][0] = p02;
    P[1][1] += c->cfg.q_bias * dt;
    P[2][2] += c->cfg.q_target * dt;
}

/* actualização escalar com H = h[3]; y = inovação; devolve ganho em k[3] */
static void kf_update(cc_t *c, const float h[3], float y, float R, float k[3])
{
    float (*P)[3] = c->P;
    float ph[3];
    for (int i = 0; i < 3; i++) ph[i] = P[i][0] * h[0] + P[i][1] * h[1] + P[i][2] * h[2];
    float S = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2] + R;
    for (int i = 0; i < 3; i++) k[i] = ph[i] / S;
    for (int i = 0; i < 3; i++) c->x[i] += k[i] * y;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) P[i][j] -= k[i] * ph[j];
    for (int i = 0; i < 3; i++)                      /* simetria e positividade */
        for (int j = i + 1; j < 3; j++) { float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = P[j][i] = m; }
    for (int i = 0; i < 3; i++) if (P[i][i] < 1e-12f) P[i][i] = 1e-12f;
}

static void finish_calibration(cc_t *c)
{
    float bias = (float)(c->gsum / c->gcount);
    float psi = c->mcount > 0 ? atan2f((float)c->msin, (float)c->mcos) : 0.0f;
    c->x[0] = psi; c->x[1] = bias; c->x[2] = psi;
    memset(c->P, 0, sizeof(c->P));
    c->P[0][0] = 0.02f * 0.02f; c->P[1][1] = 1e-6f; c->P[2][2] = 1.0f;
    c->psi_zero = psi;
    c->calibrated = 1; c->target_valid = 0; c->h_count = 0; c->h_head = 0;
    c->rate_I = 0; c->w_ref_prev = 0;
    c->mode = CC_MODE_IDLE; c->trk = CC_TRK_NONE;
}

/* ------------------------------------------------------------------ comandos */

int cc_gain_valid(uint32_t id, float v)
{
    if (id >= CC_GAIN_COUNT || !isfinite(v) || v < 0.0f) return 0;
    if (id == CC_GAIN_USE_MAG) return v == 0.0f || v == 1.0f;
    if (id == CC_GAIN_R_CAM || id == CC_GAIN_R_MAG)
        return v >= sqrtf(FLT_MIN) && v <= sqrtf(FLT_MAX);
    return 1;
}

void cc_command(cc_t *c, const cc_cmd_t *cmd)
{
    if (!c || !cmd || (cmd->type != CC_CMD_RECAL && !isfinite(cmd->a))) return;
    cc_config_t *g = &c->cfg;
    switch (cmd->type) {
    case CC_CMD_MODE:
        if (c->mode == CC_MODE_CALIB) break;
        if (cmd->a >= CC_MODE_IDLE && cmd->a <= CC_MODE_TRACK && floorf(cmd->a) == cmd->a) {
            if ((cc_mode_t)(int)cmd->a == CC_MODE_TRACK && c->mode != CC_MODE_TRACK) c->entering_track = 1;
            c->mode = (cc_mode_t)(int)cmd->a;
        }
        break;
    case CC_CMD_WHEEL:   c->wheel_sp = cmd->a; break;
    case CC_CMD_RATE:    c->rate_sp = cmd->a; break;
    case CC_CMD_HEADING: c->heading_sp = cmd->a; break;
    case CC_CMD_AIM:     c->aim = cmd->a; break;
    case CC_CMD_RECAL:
        { cc_config_t saved = c->cfg; cc_init(c, &saved); }
        break;
    case CC_CMD_GAIN:
        if (!cc_gain_valid(cmd->id, cmd->a)) break;
        switch ((cc_gain_t)cmd->id) {
        case CC_GAIN_K_THETA:    g->k_theta = cmd->a; break;
        case CC_GAIN_RATE_KP:    g->rate_kp = cmd->a; break;
        case CC_GAIN_RATE_KI:    g->rate_ki = cmd->a; break;
        case CC_GAIN_W_REF_MAX:  g->w_ref_max = cmd->a; break;
        case CC_GAIN_A_REF_MAX:  g->a_ref_max = cmd->a; break;
        case CC_GAIN_WHEEL_LIM:  g->wheel_lim = cmd->a; break;
        case CC_GAIN_TORQUE_MAX: g->torque_max = cmd->a; break;
        case CC_GAIN_R_CAM:      g->r_cam = cmd->a * cmd->a; break;
        case CC_GAIN_R_MAG:      g->r_mag = cmd->a * cmd->a; break;
        case CC_GAIN_USE_MAG:    g->use_mag = cmd->a > 0.5f; break;
        case CC_GAIN_T_SEARCH:   g->t_search = cmd->a; break;
        default: break;
        }
        break;
    default: break;
    }
}

/* -------------------------------------------------------------- sensores async */

void cc_on_mag(cc_t *c, uint64_t t_us, const float m[3])
{
    (void)t_us;
    if (!isfinite(m[0]) || !isfinite(m[1]) || !isfinite(m[2])) { c->mag_ok = 0; return; }
    float mag = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    if (fabsf(mag - c->cfg.mag_nom_uT) > c->cfg.mag_tol * c->cfg.mag_nom_uT) { c->mag_ok = 0; return; }
    if (!c->tilt_valid) return;
    if (fabsf(c->roll) > 0.7f || fabsf(c->pitch) > 0.7f) { c->mag_ok = 0; return; }

    float sr = sinf(c->roll), cr = cosf(c->roll), sp = sinf(c->pitch), cp = cosf(c->pitch);
    float xh = m[0] * cp + m[1] * sr * sp + m[2] * cr * sp;
    float yh = m[1] * cr - m[2] * sr;
    float heading = atan2f(-yh, xh);
    c->mag_ok = 1;

    if (c->mode == CC_MODE_CALIB) {
        c->msin += sinf(heading); c->mcos += cosf(heading); c->mcount++;
        return;
    }
    if (!c->calibrated || !c->cfg.use_mag) return;

    float h[3] = { 1.0f, 0.0f, 0.0f }, k[3];
    float y = wrapf(heading - c->x[0]);
    kf_update(c, h, y, c->cfg.r_mag, k);
    hist_shift(c, k[0] * y, (k[2] - k[0]) * y);
}

/* Actualização do estimador com uma detecção já feita (leve: ~microsegundos). */
int cc_on_detection(cc_t *c, uint64_t t_capture_us, uint64_t now_us, const cc_det_t *dp, cc_vision_t *out)
{
    const cc_det_t d = *dp;

    memset(out, 0, sizeof(*out));
    out->t_capture_ms = (uint32_t)(t_capture_us / 1000);
    out->found = d.found; out->n_cand = d.n_cand;
    if (!d.found || !isfinite(d.x) || !isfinite(d.y) || !isfinite(d.contrast) ||
        d.x < 0 || d.x >= CC_IMG_W || d.y < 0 || d.y >= CC_IMG_H || d.area <= 0 ||
        t_capture_us > now_us || (c->target_valid && now_us < c->last_cam_us)) return 0;
    if (c->h_count > 0) {
        int oldest = (c->h_head - c->h_count + CC_HIST) % CC_HIST;
        if (t_capture_us < c->t_hist[oldest]) return 0;
    }
    out->x = d.x; out->y = d.y; out->area = (uint32_t)d.area; out->contrast = d.contrast;
    out->err_x = atan2f(d.x - c->cfg.cam_cx, c->cfg.cam_fx);
    out->err_y = atan2f(d.y - c->cfg.cam_cy, c->cfg.cam_fy);

    if (!c->calibrated || c->mode == CC_MODE_CALIB) return 0;

    float z = out->err_x;
    double since = c->target_valid ? US(now_us - c->last_cam_us) : 1e9;

    if (since > 1.5) {                       /* (re)aquisição: alvo := psi(t_cap) + z */
        float psi_c, e_c; hist_get(c, t_capture_us, &psi_c, &e_c);
        c->x[2] = psi_c + z;
        c->P[2][2] = c->P[0][0] + c->cfg.r_cam;
        c->P[0][2] = c->P[2][0] = c->P[0][0];
        c->P[1][2] = c->P[2][1] = c->P[0][1];
        for (int k = 0; k < c->h_count; k++) c->e_hist[k] = c->x[2] - c->psi_hist[k];
        c->target_valid = 1; c->rejects = 0; c->last_cam_us = now_us;
        out->accepted = 1;
        return 1;
    }

    float psi_c, e_c; hist_get(c, t_capture_us, &psi_c, &e_c);
    float y = z - e_c;
    if (fabsf(y) > c->cfg.cam_gate) {
        if (++c->rejects >= 5) {             /* 5 rejeições seguidas: o modelo perdeu-se */
            c->target_valid = 0;
        }
        return 0;
    }
    c->rejects = 0;
    float h[3] = { -1.0f, 0.0f, 1.0f }, k[3];
    kf_update(c, h, y, c->cfg.r_cam, k);
    hist_shift(c, k[0] * y, (k[2] - k[0]) * y);
    c->last_cam_us = now_us;
    out->accepted = 1;
    return 1;
}

int cc_on_image(cc_t *c, uint64_t t_capture_us, uint64_t now_us, const uint8_t *gray, cc_vision_t *out)
{
    cc_det_t d;
    cc_detect(&c->cfg.det, &c->ws, gray, &d);
    return cc_on_detection(c, t_capture_us, now_us, &d, out);
}

/* ------------------------------------------------------------------- passo */

void cc_step(cc_t *c, uint64_t now_us, const cc_meas_t *m, cc_out_t *out)
{
    const cc_config_t *g = &c->cfg;
    /* Invalid IMU samples and non-monotonic timestamps must not update the filter. */
    if (!isfinite(m->gyro[2]) || !isfinite(m->wheel_rps) ||
        !isfinite(m->accel[0]) || !isfinite(m->accel[1]) || !isfinite(m->accel[2]) ||
        (c->started && now_us <= c->last_us)) {
        memset(out, 0, sizeof(*out));
        c->tlm.wheel_cmd_rps = 0.0f;
        return;
    }
    float dt = 0.005f;
    if (!c->started) {
        c->started = 1; c->calib_start_us = now_us; c->last_us = now_us;
    } else {
        dt = clampf((float)US(now_us - c->last_us), 1e-4f, 0.05f);
        c->last_us = now_us;
    }
    float gz = m->gyro[2];
    c->gyro_z = gz;
    c->wheel_meas = m->wheel_rps;

    /* inclinação (só serve para compensar o magnetómetro) */
    float roll_m  = atan2f(m->accel[1], m->accel[2]);
    float pitch_m = atan2f(-m->accel[0], sqrtf(m->accel[1] * m->accel[1] + m->accel[2] * m->accel[2]));
    if (!c->tilt_valid) { c->roll = roll_m; c->pitch = pitch_m; c->tilt_valid = 1; }
    else { c->roll += 0.05f * (roll_m - c->roll); c->pitch += 0.05f * (pitch_m - c->pitch); }

    out->enable = 0; out->wheel_cmd_rps = 0.0f;
    out->torque_max_nm = g->torque_max; out->accel_limit_rps2 = g->wheel_accel;

    float w_ref = 0.0f, err = 0.0f, w_hat = gz - c->x[1];

    if (c->mode == CC_MODE_CALIB) {
        c->gsum += gz; c->gsq += (double)gz * gz; c->gcount++;
        if (US(now_us - c->calib_start_us) >= g->calib_s && c->gcount > 10) {
            double mean = c->gsum / c->gcount;
            double var = c->gsq / c->gcount - mean * mean;
            if (var > 0.03 * 0.03) {          /* o cubo mexeu-se: recomeça */
                c->gsum = c->gsq = 0; c->gcount = 0; c->msin = c->mcos = 0; c->mcount = 0;
                c->calib_start_us = now_us;
            } else {
                finish_calibration(c);
            }
        }
    } else {
        kf_predict(c, dt, gz);
        hist_push(c, now_us);
        w_hat = gz - c->x[1];

        if (c->entering_track) { c->entering_track = 0; c->track_entry_us = now_us; }

        int closed_loop = 0;
        switch (c->mode) {
        case CC_MODE_WHEEL: {
            out->enable = 1;
            out->wheel_cmd_rps = clampf(c->wheel_sp, -g->wheel_lim, g->wheel_lim);
            break;
        }
        case CC_MODE_RATE:
            closed_loop = 1; w_ref = c->rate_sp;
            break;
        case CC_MODE_HEADING:
            closed_loop = 1;
            err = wrapf(c->heading_sp - (c->x[0] - c->psi_zero));
            w_ref = g->k_theta * err;
            break;
        case CC_MODE_TRACK: {
            closed_loop = 1;
            double since = c->target_valid ? US(now_us - c->last_cam_us) : US(now_us - c->track_entry_us);
            if (c->target_valid && since < g->t_lock) c->trk = CC_TRK_LOCKED;
            else if (c->target_valid && since < g->t_search) c->trk = CC_TRK_COAST;
            else if (!c->target_valid && since < g->t_search) c->trk = CC_TRK_NONE;
            else c->trk = CC_TRK_SEARCH;

            if (c->trk == CC_TRK_LOCKED || c->trk == CC_TRK_COAST) {
                err = (c->x[2] - c->x[0]) - c->aim;
                w_ref = g->k_theta * err;
                c->search_dir = err >= 0 ? 1.0f : -1.0f;
            } else if (c->trk == CC_TRK_SEARCH) {
                w_ref = c->search_dir * g->search_rate;
            }
            break;
        }
        default: break;  /* IDLE */
        }

        if (closed_loop) {
            out->enable = 1;
            w_ref = clampf(w_ref, -g->w_ref_max, g->w_ref_max);
            float step = g->a_ref_max * dt;
            c->w_ref_prev += clampf(w_ref - c->w_ref_prev, -step, step);
            float e_w = c->w_ref_prev - w_hat;

            /* PI com integração condicional (anti-windup): a roda diminui quando o corpo acelera */
            if (g->rate_ki == 0.0f) c->rate_I = 0.0f;
            float I_new = g->rate_ki > 0.0f ? c->rate_I + e_w * dt : 0.0f;
            float cmd_old = -(g->rate_kp * (e_w) + g->rate_ki * c->rate_I);
            float cmd_new = -(g->rate_kp * (e_w) + g->rate_ki * I_new);
            if (fabsf(cmd_new) <= g->wheel_lim || fabsf(cmd_new) < fabsf(cmd_old)) c->rate_I = I_new;
            out->wheel_cmd_rps = clampf(-(g->rate_kp * e_w + g->rate_ki * c->rate_I), -g->wheel_lim, g->wheel_lim);
        } else {
            /* seguimento sem salto (bumpless) para quando se fechar a malha */
            float w = (c->mode == CC_MODE_WHEEL) ? out->wheel_cmd_rps : m->wheel_rps;
            c->rate_I = g->rate_ki > 0.0f ? -w / g->rate_ki : 0.0f;
            c->w_ref_prev = w_hat;
            if (c->mode != CC_MODE_TRACK) c->trk = CC_TRK_NONE;
        }
        c->w_ref_last = w_ref;
    }

    /* telemetria */
    cc_tlm_t *t = &c->tlm;
    t->t_ms = (uint32_t)(now_us / 1000);
    t->mode = (uint8_t)c->mode; t->trk = (uint8_t)c->trk;
    t->flags = (uint16_t)((c->target_valid ? CC_FLAG_TARGET_VALID : 0)
             | ((fabsf(m->wheel_rps) > 0.95f * g->wheel_lim) ? CC_FLAG_WHEEL_SAT : 0)
             | (c->mag_ok ? CC_FLAG_MAG_OK : 0)
             | (c->calibrated ? CC_FLAG_CALIBRATED : 0)
             | ((fabsf(c->roll) > 0.17f || fabsf(c->pitch) > 0.17f) ? CC_FLAG_TILT_HIGH : 0));
    t->heading = c->x[0] - c->psi_zero;
    t->rate = w_hat; t->bias = c->x[1];
    t->err = err; t->rate_ref = c->w_ref_last;
    t->wheel_rps = m->wheel_rps; t->wheel_cmd_rps = out->wheel_cmd_rps;
    t->torque_nm = m->wheel_torque; t->vbat_v = m->vbat_v;
    t->target_err = c->x[2] - c->x[0];
    t->cam_age_s = c->target_valid ? (float)US(now_us - c->last_cam_us) : -1.0f;
}

void cc_get_tlm(const cc_t *c, cc_tlm_t *out) { *out = c->tlm; }

/* ------------------------------------------------------------ ligação ctypes */
cc_t *cc_create(void) { cc_t *c = (cc_t *)malloc(sizeof(cc_t)); if (c) cc_init(c, NULL); return c; }
void  cc_destroy(cc_t *c) { free(c); }
uint32_t cc_sizeof_cmd(void)    { return (uint32_t)sizeof(cc_cmd_t); }
uint32_t cc_sizeof_meas(void)   { return (uint32_t)sizeof(cc_meas_t); }
uint32_t cc_sizeof_out(void)    { return (uint32_t)sizeof(cc_out_t); }
uint32_t cc_sizeof_tlm(void)    { return (uint32_t)sizeof(cc_tlm_t); }
uint32_t cc_sizeof_vision(void) { return (uint32_t)sizeof(cc_vision_t); }
