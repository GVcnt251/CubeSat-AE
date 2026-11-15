#include "cube_core3.h"
#include <math.h>
#include <float.h>
#include <string.h>
#define PI 3.14159265358979323846f
static float limit(float x, float lo, float hi) {
  return fminf(hi, fmaxf(lo, x));
}
static float wrap(float a) { return remainderf(a, 2 * PI); }
static int finite3(const float a[3]) {
  return a && isfinite(a[0]) && isfinite(a[1]) && isfinite(a[2]);
}
static float norm3(const float a[3]) {
  return hypotf(hypotf(a[0], a[1]), a[2]);
}
static int unit3(float a[3]) {
  float n = norm3(a);
  if (!isfinite(n) || n < 1e-8f)
    return 0;
  for (int i = 0; i < 3; i++)
    a[i] /= n;
  return 1;
}
static void cross(const float a[3], const float b[3], float o[3]) {
  float v[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
  memcpy(o, v, sizeof(v));
}
static void mul(const float a[4], const float b[4], float o[4]) {
  float v[4] = {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
  memcpy(o, v, sizeof(v));
}
static void conjugate(const float q[4], float o[4]) {
  o[0] = q[0];
  for (int i = 1; i < 4; i++)
    o[i] = -q[i];
}
static void normalize(float q[4]) {
  float n = hypotf(hypotf(q[0], q[1]), hypotf(q[2], q[3]));
  for (int i = 0; i < 4; i++)
    q[i] /= n;
}
void cc3_quat_from_euler(const float a[3], float q[4]) {
  float cr = cosf(a[0] / 2), sr = sinf(a[0] / 2), cp = cosf(a[1] / 2),
        sp = sinf(a[1] / 2), cy = cosf(a[2] / 2), sy = sinf(a[2] / 2);
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}
void cc3_quat_to_euler(const float q[4], float a[3]) {
  a[0] = atan2f(2 * (q[0] * q[1] + q[2] * q[3]),
                1 - 2 * (q[1] * q[1] + q[2] * q[2]));
  a[1] = asinf(limit(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1));
  a[2] = atan2f(2 * (q[0] * q[3] + q[1] * q[2]),
                1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}
void cc3_rotate(const float q[4], const float v[3], float o[3]) {
  float u[3] = {q[1], q[2], q[3]}, t[3], s[3];
  cross(u, v, t);
  for (int i = 0; i < 3; i++)
    t[i] *= 2;
  cross(u, t, s);
  for (int i = 0; i < 3; i++)
    o[i] = v[i] + q[0] * t[i] + s[i];
}
static void unrotate(const float q[4], const float v[3], float o[3]) {
  float inv[4];
  conjugate(q, inv);
  cc3_rotate(inv, v, o);
}
static void advance(float q[4], const float w[3], float dt) {
  float n = norm3(w), half = n * dt / 2, d[4] = {cosf(half), 0, 0, 0};
  float scale = n > 1e-8f ? sinf(half) / n : dt / 2;
  for (int i = 0; i < 3; i++)
    d[i + 1] = w[i] * scale;
  mul(q, d, q);
  normalize(q);
}
static void rot_error(const float q[4], const float desired[4], float e[3]) {
  float inv[4], d[4];
  conjugate(q, inv);
  mul(inv, desired, d);
  if (d[0] < 0)
    for (int i = 0; i < 4; i++)
      d[i] = -d[i];
  float n = norm3(d + 1), k = n > 1e-7f ? 2 * atan2f(n, d[0]) / n : 2;
  for (int i = 0; i < 3; i++)
    e[i] = d[i + 1] * k;
}
static void axis_config_default(cc_config_t *g)
{
    memset(g, 0, sizeof(*g));
    /* câmara: 320x240, HFOV ~65.5 deg (do slide: 83.9 px <-> 18.6 deg => f = 249 px) */
    g->cam_fx = 249.0f; g->cam_fy = 249.0f; g->cam_cx = 159.5f; g->cam_cy = 119.5f;
    cc_det_cfg_default(&g->det);
    g->calib_s = 1.5f;
    g->imu_timeout_us = 50000;

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
    g->search_span_known = 0.349066f; g->search_span_unknown = 2.792527f;
}

int cc3_gain_valid(uint32_t id, float v)
{
    if (id >= CC_GAIN_COUNT || !isfinite(v) || v < 0.0f) return 0;
    if (id == CC_GAIN_USE_MAG) return v == 0.0f || v == 1.0f;
    if (id == CC_GAIN_R_CAM || id == CC_GAIN_R_MAG)
        return (double)v * v >= FLT_MIN && (double)v * v <= FLT_MAX;
    return 1;
}

void cc3_config_default(cc3_config_t *g) {
  memset(g, 0, sizeof(*g));
  for (int i = 0; i < 3; i++)
    axis_config_default(&g->axis[i]);
  g->accel_kp = 1;
  g->mag_kp = 1;
  g->gravity = 9.80665f;
  g->accel_tolerance = .2f;
  g->gyro_limit = 30;
  g->stationary_limit = .05f;
}
static int positive(float v, float max) {
  return isfinite(v) && v > 0 && v <= max;
}
static int config_ok(const cc3_config_t *g) {
  if (!positive(g->gravity, 20) || !positive(g->accel_tolerance, .5f) ||
      !positive(g->gyro_limit, 100) || !positive(g->stationary_limit, 1) ||
      g->stationary_limit > g->gyro_limit || !isfinite(g->accel_kp) ||
      g->accel_kp < 0 || g->accel_kp > 10 || !isfinite(g->mag_kp) ||
      g->mag_kp < 0 || g->mag_kp > 10)
    return 0;
  for (int i = 0; i < 3; i++) {
    const cc_config_t *a = &g->axis[i];
    if (!isfinite(a->k_theta) || a->k_theta < 0 || a->k_theta > 100 ||
        !isfinite(a->rate_kp) || a->rate_kp < 0 || a->rate_kp > 10000 ||
        !isfinite(a->rate_ki) || a->rate_ki < 0 || a->rate_ki > 10000 ||
        !positive(a->w_ref_max, 20) || !positive(a->a_ref_max, 100) ||
        !positive(a->wheel_lim, 10000) || !positive(a->wheel_accel, 100000) ||
        !positive(a->torque_max, 100) || a->gimbal_unload != 0)
      return 0;
  }
  const cc_config_t *a = &g->axis[2];
  return (a->use_mag == 0 || a->use_mag == 1) && positive(a->cam_fx, 100000) &&
         positive(a->cam_fy, 100000) && isfinite(a->cam_cx) &&
         isfinite(a->cam_cy) && positive(a->calib_s, 60) &&
         a->imu_timeout_us > 0 && a->imu_timeout_us <= 500000 &&
         positive(a->cam_gate, PI) && positive(a->r_cam, 100) &&
         positive(a->r_mag, 100) && positive(a->mag_nom_uT, 1000) &&
         positive(a->mag_tol, 1) && positive(a->t_lock, 60) &&
         positive(a->t_search, 300) && a->t_search >= a->t_lock &&
         positive(a->search_rate, 10) && positive(a->search_span_known, PI) &&
         positive(a->search_span_unknown, PI) &&
         a->search_span_unknown >= a->search_span_known;
}
int cc3_init(cc3_t *c, const cc3_config_t *cfg) {
  if (!c)
    return 0;
  cc3_config_t g;
  if (cfg)
    g = *cfg;
  else
    cc3_config_default(&g);
  if (!config_ok(&g))
    return 0;
  memset(c, 0, sizeof(*c));
  c->cfg = g;
  c->q[0] = c->q_zero[0] = 1;
  c->mode = CC_MODE_CALIB;
  c->search_dir = 1;
  for (int i = 0; i < 3; i++)
    c->tlm.axis[i].cam_age_s = -1;
  return 1;
}
static void reset_window(cc3_t *c) {
  c->calib_count = c->mag_count = 0;
  memset(c->gyro_sum, 0, sizeof(c->gyro_sum));
  memset(c->gyro_sq, 0, sizeof(c->gyro_sq));
  memset(c->accel_sum, 0, sizeof(c->accel_sum));
  memset(c->mag_sum, 0, sizeof(c->mag_sum));
}
int cc3_command_vector(cc3_t *c, uint32_t kind, const float v[3]) {
  if (!c || !finite3(v))
    return 0;
  float *dest = NULL;
  switch (kind) {
  case CC_CMD_WHEEL:
    dest = c->wheel_sp;
    for (int i = 0; i < 3; i++)
      if (fabsf(v[i]) > c->cfg.axis[i].wheel_lim)
        return 0;
    break;
  case CC_CMD_RATE:
    dest = c->rate_sp;
    for (int i = 0; i < 3; i++)
      if (fabsf(v[i]) > c->cfg.axis[i].w_ref_max)
        return 0;
    break;
  case CC_CMD_HEADING:
    dest = c->attitude_sp;
    if (fabsf(v[0]) > PI || fabsf(v[1]) > PI / 2 || fabsf(v[2]) > PI)
      return 0;
    break;
  case CC_CMD_AIM:
    dest = c->aim;
    if (fabsf(v[0]) > PI || fabsf(v[1]) > PI / 3 || fabsf(v[2]) > PI / 3)
      return 0;
    break;
  default:
    return 0;
  }
  memcpy(dest, v, 3 * sizeof(float));
  return 1;
}
int cc3_set_gain(cc3_t *c, unsigned axis, uint32_t id, float value) {
  if (!c || axis >= 3 || !cc3_gain_valid(id, value))
    return 0;
  cc3_config_t g = c->cfg;
  cc_config_t *a = &g.axis[axis];
  switch (id) {
  case CC_GAIN_K_THETA:
    a->k_theta = value;
    break;
  case CC_GAIN_RATE_KP:
    a->rate_kp = value;
    break;
  case CC_GAIN_RATE_KI:
    a->rate_ki = value;
    break;
  case CC_GAIN_W_REF_MAX:
    a->w_ref_max = value;
    break;
  case CC_GAIN_A_REF_MAX:
    a->a_ref_max = value;
    break;
  case CC_GAIN_WHEEL_LIM:
    a->wheel_lim = value;
    break;
  case CC_GAIN_TORQUE_MAX:
    a->torque_max = value;
    break;
  case CC_GAIN_USE_MAG:
    if (axis != 2)
      return 0;
    a->use_mag = (int)value;
    break;
  case CC_GAIN_T_SEARCH:
    if (axis != 2)
      return 0;
    a->t_search = value;
    break;
  case CC_GAIN_R_CAM:
    if (axis != 2)
      return 0;
    a->r_cam = value * value;
    break;
  case CC_GAIN_R_MAG:
    if (axis != 2)
      return 0;
    a->r_mag = value * value;
    break;
  default:
    return 0;
  }
  if (!config_ok(&g))
    return 0;
  c->cfg = g;
  return 1;
}
int cc3_command_legacy(cc3_t *c, const cc_cmd_t *cmd) {
  if (!c || !cmd)
    return 0;
  if (cmd->type == CC_CMD_RECAL) {
    c->mode = CC_MODE_CALIB;
    c->calibrated = 0;
    c->recal_requested = 1;
    reset_window(c);
    c->target_valid = c->search_known = 0;
    c->count = c->head = 0;
    c->mag_ok = 0;
    memset(c->integral, 0, sizeof(c->integral));
    memset(c->ref, 0, sizeof(c->ref));
    memset(c->wheel_sp, 0, sizeof(c->wheel_sp));
    memset(c->rate_sp, 0, sizeof(c->rate_sp));
    memset(c->attitude_sp, 0, sizeof(c->attitude_sp));
    memset(c->aim, 0, sizeof(c->aim));
    return 1;
  }
  if (!isfinite(cmd->a))
    return 0;
  if (cmd->type == CC_CMD_MODE) {
    if (c->mode == CC_MODE_CALIB || cmd->a < 1 || cmd->a > 5 ||
        floorf(cmd->a) != cmd->a)
      return 0;
    c->mode = (cc_mode_t)(int)cmd->a;
    c->trk = CC_TRK_NONE;
    c->track_entry = c->last_call;
    memset(c->integral, 0, sizeof(c->integral));
    memset(c->ref, 0, sizeof(c->ref));
    return 1;
  }
  if (cmd->type == CC_CMD_GAIN) {
    cc3_config_t saved = c->cfg;
    if (cmd->id == CC_GAIN_USE_MAG || cmd->id == CC_GAIN_T_SEARCH ||
        cmd->id == CC_GAIN_R_CAM || cmd->id == CC_GAIN_R_MAG)
      return cc3_set_gain(c, 2, cmd->id, cmd->a);
    for (unsigned i = 0; i < 3; i++)
      if (!cc3_set_gain(c, i, cmd->id, cmd->a)) {
        c->cfg = saved;
        return 0;
      }
    return 1;
  }
  if (cmd->type == CC_CMD_HEADING) {
    /* Old scalar heading is world yaw relative to initial yaw. Preserve the
     * current desired world roll/pitch and convert back into the 3D reference.
     * With a tilted calibration frame this can change several relative Euler
     * components; simply writing attitude_sp[2] would change its meaning. */
    float relative[4], desired[4], inverse[4], rpy[3], zero[3], v[3];
    cc3_quat_from_euler(c->attitude_sp, relative);
    mul(c->q_zero, relative, desired);
    cc3_quat_to_euler(desired, rpy);
    cc3_quat_to_euler(c->q_zero, zero);
    rpy[2] = wrap(zero[2] + wrap(cmd->a));
    cc3_quat_from_euler(rpy, desired);
    conjugate(c->q_zero, inverse);
    mul(inverse, desired, relative);
    cc3_quat_to_euler(relative, v);
    return cc3_command_vector(c, CC_CMD_HEADING, v);
  }
  const float *base = cmd->type == CC_CMD_WHEEL     ? c->wheel_sp
                      : cmd->type == CC_CMD_RATE    ? c->rate_sp
                      : cmd->type == CC_CMD_HEADING ? c->attitude_sp
                                                    : c->aim;
  float v[3] = {base[0], base[1], cmd->a};
  return cc3_command_vector(c, cmd->type, v);
}
static int accel_ok(const cc3_t *c, const float a[3]) {
  return finite3(a) && fabsf(norm3(a) - c->cfg.gravity) <=
                           c->cfg.accel_tolerance * c->cfg.gravity;
}
static int mag_ok(const cc3_t *c, const cc3_meas_t *m) {
  const cc_config_t *g = &c->cfg.axis[2];
  return m->mag_valid && finite3(m->mag) &&
         fabsf(norm3(m->mag) - g->mag_nom_uT) <= g->mag_tol * g->mag_nom_uT;
}
static float mag_heading(const float rpy[3], const float m[3]) {
  float sr = sinf(rpy[0]), cr = cosf(rpy[0]), sp = sinf(rpy[1]),
        cp = cosf(rpy[1]);
  return atan2f(-(m[1] * cr - m[2] * sr),
                m[0] * cp + m[1] * sr * sp + m[2] * cr * sp);
}
static void calibration(cc3_t *c, uint64_t t, const cc3_meas_t *m) {
  if (!accel_ok(c, m->accel) || norm3(m->gyro) > c->cfg.stationary_limit) {
    reset_window(c);
    return;
  }
  if (!c->calib_count)
    c->calib_start = t;
  c->calib_count++;
  for (int i = 0; i < 3; i++) {
    c->gyro_sum[i] += m->gyro[i];
    c->gyro_sq[i] += (double)m->gyro[i] * m->gyro[i];
    c->accel_sum[i] += m->accel[i];
  }
  if (mag_ok(c, m)) {
    c->mag_count++;
    for (int i = 0; i < 3; i++)
      c->mag_sum[i] += m->mag[i];
  }
  if ((t - c->calib_start) * 1e-6 < c->cfg.axis[2].calib_s ||
      c->calib_count < 20)
    return;
  float a[3], mag[3], rpy[3];
  for (int i = 0; i < 3; i++) {
    double mean = c->gyro_sum[i] / c->calib_count;
    if (c->gyro_sq[i] / c->calib_count - mean * mean > .03 * .03) {
      reset_window(c);
      return;
    }
    c->bias[i] = (float)mean;
    a[i] = (float)(c->accel_sum[i] / c->calib_count);
    mag[i] = (float)(c->mag_sum[i] / (c->mag_count ? c->mag_count : 1));
  }
  rpy[0] = atan2f(a[1], a[2]);
  rpy[1] = atan2f(-a[0], hypotf(a[1], a[2]));
  rpy[2] = 0;
  c->mag_reference_valid = c->mag_count && c->cfg.axis[2].use_mag;
  if (c->mag_reference_valid)
    rpy[2] = mag_heading(rpy, mag);
  cc3_quat_from_euler(rpy, c->q);
  memcpy(c->q_zero, c->q, sizeof(c->q));
  c->calibrated = c->calibrated_once = 1;
  c->mode = CC_MODE_IDLE;
  c->trk = CC_TRK_NONE;
  c->target_valid = c->search_known = 0;
  c->count = c->head = 0;
  memset(c->attitude_sp, 0, sizeof(c->attitude_sp));
  memset(c->integral, 0, sizeof(c->integral));
  memset(c->ref, 0, sizeof(c->ref));
  if (c->recal_requested)
    c->blocked = 0;
  c->recal_requested = 0;
}
static void hist_push(cc3_t *c, uint64_t t) {
  c->hist_t[c->head] = t;
  memcpy(c->hist_q[c->head], c->q, sizeof(c->q));
  c->head = (c->head + 1) % CC_HIST;
  if (c->count < CC_HIST)
    c->count++;
}
static int hist_get(const cc3_t *c, uint64_t t, float q[4]) {
  if (!c->count)
    return 0;
  unsigned start = (c->head + CC_HIST - c->count) % CC_HIST,
           newest = (c->head + CC_HIST - 1) % CC_HIST;
  if (t < c->hist_t[start] || t > c->hist_t[newest])
    return 0;
  for (unsigned j = 0; j < c->count; j++) {
    unsigned k = (start + j) % CC_HIST;
    if (t == c->hist_t[k]) {
      memcpy(q, c->hist_q[k], 4 * sizeof(float));
      return 1;
    }
    if (t < c->hist_t[k]) {
      unsigned prev = (k + CC_HIST - 1) % CC_HIST;
      float f = (float)((double)(t - c->hist_t[prev]) /
                        (c->hist_t[k] - c->hist_t[prev])),
            dot = 0;
      for (int i = 0; i < 4; i++)
        dot += c->hist_q[prev][i] * c->hist_q[k][i];
      for (int i = 0; i < 4; i++)
        q[i] = (1 - f) * c->hist_q[prev][i] +
               f * (dot < 0 ? -1 : 1) * c->hist_q[k][i];
      normalize(q);
      return 1;
    }
  }
  return 0;
}
int cc3_on_detection(cc3_t *c, uint64_t capture, uint64_t now,
                     const cc_det_t *d, cc_vision_t *out) {
  if (!out)
    return 0;
  memset(out, 0, sizeof(*out));
  if (!c || !d)
    return 0;
  out->t_capture_ms = (uint32_t)(capture / 1000);
  out->found = d->found;
  out->n_cand = d->n_cand;
  if (!d->found || !isfinite(d->x) || !isfinite(d->y) ||
      !isfinite(d->contrast) || d->x < 0 || d->x >= CC_IMG_W || d->y < 0 ||
      d->y >= CC_IMG_H || d->area <= 0 || d->area > CC_IMG_N)
    return 0;
  const cc_config_t *g = &c->cfg.axis[2];
  out->x = d->x;
  out->y = d->y;
  out->contrast = d->contrast;
  out->area = (uint32_t)d->area;
  float ray[3] = {1, (d->x - g->cam_cx) / g->cam_fx,
                  (d->y - g->cam_cy) / g->cam_fy},
        q[4], world[3];
  out->err_x = atan2f(ray[1], 1);
  out->err_y = atan2f(ray[2], 1);
  if (!c->calibrated || c->mode == CC_MODE_CALIB || c->missing || c->blocked ||
      capture > now || now < c->last_valid ||
      now - c->last_valid > g->imu_timeout_us ||
      (c->cam_seen && (capture <= c->last_capture || now < c->last_cam)) ||
      !hist_get(c, capture, q))
    return 0;
  unit3(ray);
  cc3_rotate(q, ray, world);
  if (c->target_valid && (now - c->last_cam) * 1e-6 <= 1.5) {
    float dot = 0;
    for (int i = 0; i < 3; i++)
      dot += world[i] * c->target_world[i];
    if (acosf(limit(dot, -1, 1)) > g->cam_gate) {
      if (++c->rejects >= 5)
        c->target_valid = 0;
      return 0;
    }
    /* Complementary confidence weight against a fixed nominal prior variance.
     * Same sigma units as the ICD, not a Kalman covariance estimate. */
    float blend = .000049f / (.000049f + g->r_cam);
    for (int i = 0; i < 3; i++)
      world[i] = blend * world[i] + (1 - blend) * c->target_world[i];
    unit3(world);
  }
  memcpy(c->target_world, world, sizeof(world));
  c->target_valid = c->search_known = c->cam_seen = 1;
  c->rejects = 0;
  c->last_cam = now;
  c->last_capture = capture;
  out->accepted = 1;
  return 1;
}
static void estimator(cc3_t *c, const cc3_meas_t *m, float dt) {
  float w[3];
  for (int i = 0; i < 3; i++)
    c->rate[i] = w[i] = m->gyro[i] - c->bias[i];
  if (accel_ok(c, m->accel)) {
    float a[3], down[3] = {0, 0, 1}, pred[3], e[3];
    memcpy(a, m->accel, sizeof(a));
    unit3(a);
    unrotate(c->q, down, pred);
    cross(a, pred, e);
    for (int i = 0; i < 3; i++)
      w[i] += c->cfg.accel_kp * e[i];
  }
  c->mag_ok = 0;
  if (c->mag_reference_valid && c->cfg.axis[2].use_mag && mag_ok(c, m)) {
    float rpy[3], down[3] = {0, 0, 1}, axis[3];
    cc3_quat_to_euler(c->q, rpy);
    /* Tilt-compensated compass is ill-conditioned near vertical pitch. */
    if (fabsf(cosf(rpy[1])) > .15f) {
      float e = wrap(mag_heading(rpy, m->mag) - rpy[2]);
      unrotate(c->q, down, axis);
      for (int i = 0; i < 3; i++)
        w[i] += c->cfg.mag_kp * (.005f / (.0025f + c->cfg.axis[2].r_mag)) * e *
                axis[i];
      c->mag_ok = 1;
    }
  }
  advance(c->q, w, dt);
}
static void desired_attitude(cc3_t *c, uint64_t now, float desired[4]) {
  float rel[4];
  cc3_quat_from_euler(c->attitude_sp, rel);
  mul(c->q_zero, rel, desired);
  c->trk = CC_TRK_NONE;
  if (c->mode != CC_MODE_TRACK)
    return;
  double age = c->target_valid ? (now - c->last_cam) * 1e-6
                               : (now - c->track_entry) * 1e-6;
  const cc_config_t *g = &c->cfg.axis[2];
  cc_trk_t state = c->target_valid && age < g->t_lock     ? CC_TRK_LOCKED
                   : c->target_valid && age < g->t_search ? CC_TRK_COAST
                   : age < g->t_search                    ? CC_TRK_NONE
                                                          : CC_TRK_SEARCH;
  c->trk = state;
  if (state == CC_TRK_LOCKED || state == CC_TRK_COAST) {
    float rpy[3] = {c->aim[0],
                    atan2f(-c->target_world[2],
                           hypotf(c->target_world[0], c->target_world[1])),
                    atan2f(c->target_world[1], c->target_world[0])};
    /* Build look direction first; roll around it. Offset is a body-camera ray.
     */
    float aim[3] = {0, atan2f(-tanf(c->aim[1]), hypotf(1, tanf(c->aim[2]))),
                    c->aim[2]},
          qa[4], inv[4];
    cc3_quat_from_euler(rpy, desired);
    cc3_quat_from_euler(aim, qa);
    conjugate(qa, inv);
    mul(desired, inv, desired);
  }
}
static void telemetry(cc3_t *c, uint64_t now, const cc3_meas_t *m,
                      const cc3_out_t *o, cc_imu_status_t status) {
  float inv[4], rel[4], angles[3], world_angles[3];
  conjugate(c->q_zero, inv);
  mul(inv, c->q, rel);
  cc3_quat_to_euler(rel, angles);
  cc3_quat_to_euler(c->q, world_angles);
  c->tlm.imu_status = (uint8_t)status;
  c->tlm.motor_valid_mask = 0;
  if (m)
    for (int i = 0; i < 3; i++)
      if ((m->motor_valid_mask & (1u << i)) && isfinite(m->wheel_rps[i]))
        c->tlm.motor_valid_mask |= (uint8_t)(1u << i);
  float ray[3] = {1, 0, 0};
  if (c->target_valid)
    unrotate(c->q, c->target_world, ray);
  for (int i = 0; i < 3; i++) {
    cc_tlm_t *t = &c->tlm.axis[i];
    t->t_ms = (uint32_t)(now / 1000);
    t->mode = (uint8_t)c->mode;
    t->trk = (uint8_t)c->trk;
    t->heading = angles[i];
    t->rate = c->rate[i];
    t->bias = c->bias[i];
    t->err = c->err[i];
    t->rate_ref = c->ref[i];
    if (m) {
      if (isfinite(m->wheel_rps[i]))
        t->wheel_rps = m->wheel_rps[i];
      if (isfinite(m->wheel_torque[i]))
        t->torque_nm = m->wheel_torque[i];
      if (isfinite(m->vbat_v))
        t->vbat_v = m->vbat_v;
    }
    t->wheel_cmd_rps = o->axis[i].wheel_cmd_rps;
    t->cam_age_s =
        c->cam_seen
            ? (float)((now >= c->last_cam ? now - c->last_cam : 0) * 1e-6)
            : -1;
    t->target_err = i == 2   ? atan2f(ray[1], ray[0])
                    : i == 1 ? atan2f(-ray[2], hypotf(ray[0], ray[1]))
                             : 0;
    t->flags = (uint16_t)((c->target_valid ? CC_FLAG_TARGET_VALID : 0) |
                          (c->calibrated ? CC_FLAG_CALIBRATED : 0) |
                          (c->mag_ok ? CC_FLAG_MAG_OK : 0) |
                          (fabsf(world_angles[0]) > PI / 18 ||
                                   fabsf(world_angles[1]) > PI / 18
                               ? CC_FLAG_TILT_HIGH
                               : 0) |
                          (fabsf(t->wheel_rps) > .95f * c->cfg.axis[i].wheel_lim
                               ? CC_FLAG_WHEEL_SAT
                               : 0));
  }
}
cc_imu_status_t cc3_step_with_imu(cc3_t *c, uint64_t now, const cc3_meas_t *m,
                                  int imu_valid, cc3_out_t *out) {
  if (!out)
    return CC_IMU_UNAVAILABLE;
  memset(out, 0, sizeof(*out));
  if (!c)
    return CC_IMU_UNAVAILABLE;
  for (int i = 0; i < 3; i++) {
    out->axis[i].torque_max_nm = c->cfg.axis[i].torque_max;
    out->axis[i].accel_limit_rps2 = c->cfg.axis[i].wheel_accel;
  }
  int monotonic = !c->call_seen || now > c->last_call;
  int valid = imu_valid && m && monotonic && finite3(m->gyro) &&
              finite3(m->accel) && norm3(m->gyro) <= c->cfg.gyro_limit;
  int gap = c->imu_seen && now >= c->last_valid &&
            now - c->last_valid > c->cfg.axis[2].imu_timeout_us;
  int recovering = c->missing || gap;
  float dt = c->imu_seen && now > c->last_valid
                 ? (float)((now - c->last_valid) * 1e-6)
                 : .005f;
  if (monotonic) {
    c->last_call = now;
    c->call_seen = 1;
  }
  if (gap)
    c->blocked = 1;
  if (recovering || c->blocked)
    c->mag_ok = 0;
  int normal = 0;
  if (!valid) {
    c->missing = 1;
    c->mag_ok = 0;
    if (c->mode == CC_MODE_CALIB)
      reset_window(c);
  } else {
    c->missing = 0;
    c->imu_seen = 1;
    c->last_valid = now;
    if (recovering) {
      c->count = c->head = 0;
      c->target_valid = 0;
      c->track_entry = now;
      memset(c->integral, 0, sizeof(c->integral));
      memset(c->ref, 0, sizeof(c->ref));
      if (c->mode == CC_MODE_CALIB)
        reset_window(c);
    }
    if (c->mode == CC_MODE_CALIB)
      calibration(c, now, m);
    else if (!recovering && !c->blocked) {
      estimator(c, m, dt);
      normal = 1;
    }
    if (c->calibrated && !c->blocked)
      hist_push(c, now);
  }
  cc_imu_status_t status = c->blocked ? CC_IMU_RECAL_REQUIRED
                           : valid    ? CC_IMU_OK
                                      : CC_IMU_UNAVAILABLE;
  int motors = m && (m->motor_valid_mask & 7) == 7 && finite3(m->wheel_rps);
  if (c->mode == CC_MODE_WHEEL && c->calibrated_once && m) {
    for (int i = 0; i < 3; i++)
      if ((m->motor_valid_mask & (1u << i)) && isfinite(m->wheel_rps[i])) {
        out->axis[i].enable = 1;
        out->axis[i].wheel_cmd_rps =
            limit(c->wheel_sp[i], -c->cfg.axis[i].wheel_lim,
                  c->cfg.axis[i].wheel_lim);
      }
  } else if (normal && motors &&
             (c->mode == CC_MODE_RATE || c->mode == CC_MODE_HEADING ||
              c->mode == CC_MODE_TRACK)) {
    float desired[4], ref[3];
    cc_trk_t previous = c->trk;
    desired_attitude(c, now, desired);
    rot_error(c->q, desired, c->err);
    if (c->mode == CC_MODE_RATE)
      memset(c->err, 0, sizeof(c->err));
    for (int i = 0; i < 3; i++)
      ref[i] = c->mode == CC_MODE_RATE ? c->rate_sp[i]
                                       : c->cfg.axis[i].k_theta * c->err[i];
    if (c->mode == CC_MODE_TRACK && c->trk == CC_TRK_NONE)
      memset(ref, 0, sizeof(ref));
    if (c->mode == CC_MODE_TRACK && c->trk == CC_TRK_SEARCH) {
      float rpy[3];
      cc3_quat_to_euler(c->q, rpy);
      if (previous != CC_TRK_SEARCH)
        c->search_center = c->search_known
                               ? atan2f(c->target_world[1], c->target_world[0])
                               : rpy[2];
      float span = c->search_known ? c->cfg.axis[2].search_span_known
                                   : c->cfg.axis[2].search_span_unknown;
      float delta = wrap(rpy[2] - c->search_center);
      if (delta >= span)
        c->search_dir = -1;
      else if (delta <= -span)
        c->search_dir = 1;
      /* Rotate about world down, expressed in body coordinates; hold tilt. */
      float down[3] = {0, 0, 1}, body[3];
      unrotate(c->q, down, body);
      float along = 0;
      for (int i = 0; i < 3; i++)
        along += ref[i] * body[i];
      for (int i = 0; i < 3; i++)
        ref[i] +=
            (c->search_dir * c->cfg.axis[2].search_rate - along) * body[i];
    }
    for (int i = 0; i < 3; i++) {
      const cc_config_t *g = &c->cfg.axis[i];
      ref[i] = limit(ref[i], -g->w_ref_max, g->w_ref_max);
      c->ref[i] +=
          limit(ref[i] - c->ref[i], -g->a_ref_max * dt, g->a_ref_max * dt);
      float e = c->ref[i] - c->rate[i];
      if (g->rate_ki == 0)
        c->integral[i] = 0;
      float next = limit(c->integral[i] + e * dt, -1000000, 1000000);
      float old = -g->rate_kp * e - g->rate_ki * c->integral[i],
            candidate = -g->rate_kp * e - g->rate_ki * next;
      if (g->rate_ki > 0 &&
          (fabsf(candidate) <= g->wheel_lim || fabsf(candidate) < fabsf(old)))
        c->integral[i] = next;
      out->axis[i].enable = 1;
      out->axis[i].wheel_cmd_rps =
          limit(-g->rate_kp * e - g->rate_ki * c->integral[i], -g->wheel_lim,
                g->wheel_lim);
    }
  }
  if (!normal || !motors || c->mode == CC_MODE_IDLE ||
      c->mode == CC_MODE_WHEEL || c->mode == CC_MODE_CALIB) {
    memset(c->ref, 0, sizeof(c->ref));
    memset(c->err, 0, sizeof(c->err));
    c->trk = CC_TRK_NONE;
    /* Freeze integrator on IMU outage; reset after motor loss or open-loop
     * modes. */
    if (!motors || c->mode == CC_MODE_IDLE || c->mode == CC_MODE_WHEEL)
      memset(c->integral, 0, sizeof(c->integral));
  }
  telemetry(c, now, m, out, status);
  return status;
}
void cc3_get_tlm(const cc3_t *c, cc3_tlm_t *out) {
  if (c && out)
    *out = c->tlm;
}
void cc3_get_legacy_tlm(const cc3_t *c, cc_tlm_t *out) {
  if (c && out) {
    *out = c->tlm.axis[2];
    /* Preserve the scalar ICD's heading: world yaw minus initial world yaw.
     * In a tilted frame this differs from the Z Euler angle in TM_STATE3. */
    float world[3], zero[3];
    cc3_quat_to_euler(c->q, world);
    cc3_quat_to_euler(c->q_zero, zero);
    out->heading = wrap(world[2] - zero[2]);
  }
}
void cc3_get_attitude(const cc3_t *c, float q[4]) {
  if (c && q)
    memcpy(q, c->q, 4 * sizeof(float));
}
uint32_t cc3_sizeof_state(void) { return (uint32_t)sizeof(cc3_t); }
uint32_t cc3_sizeof_meas(void) { return (uint32_t)sizeof(cc3_meas_t); }
uint32_t cc3_sizeof_out(void) { return (uint32_t)sizeof(cc3_out_t); }
uint32_t cc3_sizeof_tlm(void) { return (uint32_t)sizeof(cc3_tlm_t); }
