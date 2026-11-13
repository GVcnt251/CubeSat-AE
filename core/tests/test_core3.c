#include "cc_proto3.h"
#include "cube_core3.h"
#include "wire_le.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static int fails;
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x)) {                                                                \
      printf("FAIL %d: %s\n", __LINE__, #x);                                   \
      fails++;                                                                 \
    }                                                                          \
  } while (0)
static cc3_t c;
static cc3_meas_t m;
static cc3_out_t o;
static uint64_t t;
static void tick(int valid) {
  t += 5000;
  cc3_step_with_imu(&c, t, &m, valid, &o);
}
static void mode(int v) {
  cc_cmd_t cmd = {CC_CMD_MODE, 0, (float)v, 0};
  CHECK(cc3_command_legacy(&c, &cmd));
}
static void recal(void) {
  cc_cmd_t cmd = {CC_CMD_RECAL, 0, 0, 0};
  CHECK(cc3_command_legacy(&c, &cmd));
}
static void ready(void) {
  CHECK(cc3_init(&c, NULL));
  memset(&m, 0, sizeof(m));
  m.accel[2] = 9.80665f;
  m.mag[0] = 44;
  m.mag_valid = 1;
  m.motor_valid_mask = 7;
  t = 0;
  for (int i = 0; i < 330; i++)
    tick(1);
  CHECK(c.calibrated && c.mode == CC_MODE_IDLE);
}
static void check_calibration(void) {
  ready();
  recal();
  for (int i = 0; i < 200; i++)
    tick(1);
  tick(0);
  for (int i = 0; i < 100; i++)
    tick(1);
  CHECK(!c.calibrated);
  for (int i = 0; i < 230; i++)
    tick(1);
  CHECK(c.calibrated);
  recal();
  m.gyro[0] = .2f;
  for (int i = 0; i < 400; i++)
    tick(1);
  CHECK(!c.calibrated);
}
static void failures(void) {
  for (int v = CC_MODE_RATE; v <= CC_MODE_TRACK; v++) {
    ready();
    mode(v);
    tick(1);
    CHECK(o.axis[0].enable && o.axis[1].enable && o.axis[2].enable);
    float q[4], integral[3];
    memcpy(q, c.q, sizeof(q));
    memcpy(integral, c.integral, sizeof(integral));
    tick(0);
    CHECK(!memcmp(q, c.q, sizeof(q)) &&
          !memcmp(integral, c.integral, sizeof(integral)));
    for (int i = 0; i < 3; i++)
      CHECK(!o.axis[i].enable);
    tick(1);
    CHECK(!memcmp(q, c.q, sizeof(q)));
    CHECK(!o.axis[2].enable);
    tick(1);
    CHECK(o.axis[2].enable);
    uint64_t last = c.last_valid;
    t = last + 50000;
    CHECK(cc3_step_with_imu(&c, t, &m, 0, &o) == CC_IMU_UNAVAILABLE);
    t++;
    CHECK(cc3_step_with_imu(&c, t, &m, 1, &o) == CC_IMU_RECAL_REQUIRED);
    CHECK(!o.axis[2].enable);
    tick(1);
    CHECK(c.blocked && !o.axis[0].enable);
    mode(CC_MODE_WHEEL);
    float wheels[3] = {10, 20, 30};
    CHECK(cc3_command_vector(&c, CC_CMD_WHEEL, wheels));
    tick(0);
    for (int i = 0; i < 3; i++)
      CHECK(o.axis[i].enable && o.axis[i].wheel_cmd_rps == wheels[i]);
    recal();
    CHECK(c.blocked);
    for (int i = 0; i < 330; i++)
      tick(1);
    CHECK(!c.blocked && c.calibrated);
  }
  ready();
  mode(CC_MODE_HEADING);
  m.motor_valid_mask = 5;
  tick(1);
  CHECK(!o.axis[0].enable && !o.axis[1].enable && !o.axis[2].enable);
  mode(CC_MODE_WHEEL);
  tick(1);
  CHECK(o.axis[0].enable && !o.axis[1].enable && o.axis[2].enable);
  ready();
  mode(CC_MODE_RATE);
  m.gyro[0] = NAN;
  tick(1);
  CHECK(c.missing && !o.axis[2].enable);
  ready();
  tick(1);
  CHECK(cc3_step_with_imu(&c, t, &m, 1, &o) == CC_IMU_UNAVAILABLE);
  ready();
  mode(CC_MODE_RATE);
  m.wheel_rps[0] = NAN;
  float old = c.q[3];
  m.gyro[2] = .5f;
  tick(1);
  CHECK(c.q[3] > old && !o.axis[2].enable);
}
static void axes(void) {
  for (int axis = 0; axis < 3; axis++) {
    ready();
    float ref[3] = {0};
    ref[axis] = .3f;
    CHECK(cc3_command_vector(&c, CC_CMD_HEADING, ref));
    mode(CC_MODE_HEADING);
    tick(1);
    CHECK(o.axis[axis].wheel_cmd_rps < 0);
    for (int j = 0; j < 3; j++)
      if (j != axis)
        CHECK(fabsf(o.axis[j].wheel_cmd_rps) < 1e-4f);
  }
  ready();
  float rpy[3] = {.3f, -.4f, .5f}, q[4], back[3];
  cc3_quat_from_euler(rpy, q);
  cc3_quat_to_euler(q, back);
  for (int i = 0; i < 3; i++)
    CHECK(fabsf(back[i] - rpy[i]) < 1e-5f);
  CHECK(cc3_set_gain(&c, 0, CC_GAIN_RATE_KI, 0));
  mode(CC_MODE_RATE);
  tick(1);
  CHECK(c.integral[0] == 0);
}
static void camera(void) {
  ready();
  cc_det_t d = {0};
  cc_vision_t v;
  d.found = 1;
  d.x = 190;
  d.y = 140;
  d.area = 100;
  d.contrast = 100;
  CHECK(cc3_on_detection(&c, t, t, &d, &v));
  CHECK(v.err_x > 0 && v.err_y > 0);
  CHECK(!cc3_on_detection(&c, t, t, &d, &v));
  mode(CC_MODE_TRACK);
  tick(1);
  CHECK(o.axis[1].wheel_cmd_rps > 0 && o.axis[2].wheel_cmd_rps < 0);
  uint64_t old = t;
  tick(0);
  CHECK(!cc3_on_detection(&c, t, t, &d, &v) && v.found && !v.accepted);
  tick(1);
  CHECK(!cc3_on_detection(&c, old, t, &d, &v));
  tick(1);
  CHECK(cc3_on_detection(&c, t, t, &d, &v));
  uint64_t accepted = t;
  recal();
  for (int i = 0; i < 330; i++)
    tick(1);
  CHECK(fabsf(c.tlm.axis[2].cam_age_s - (float)((t - accepted) * 1e-6)) <
        1e-5f);
  /* Capture transformed using history, not current orientation. */
  ready();
  uint64_t capture = t;
  for (int k = 0; k < 10; k++) {
    m.gyro[2] = 1;
    tick(1);
  }
  d.x = 159.5f;
  d.y = 119.5f;
  CHECK(cc3_on_detection(&c, capture, t, &d, &v));
  CHECK(fabsf(c.target_world[1]) < 1e-5f);
  CHECK(!cc3_on_detection(&c, t + 1000, t, &d, &v));
}
static void protocol(void) {
  ready();
  uint8_t p[240] = {1, CC_CMD_HEADING};
  for (int i = 0; i < 3; i++)
    cc_write_le_float(p + 2 + 4 * i, 10.0f * (i + 1));
  CHECK(cc3_apply_tc(&c, TC_SET_VECTOR3, p, 14) == ACK_OK);
  for (int i = 0; i < 3; i++)
    CHECK(fabsf(c.attitude_sp[i] - (i + 1) * .174532925f) < 1e-6f);
  float saved[3];
  memcpy(saved, c.attitude_sp, sizeof(saved));
  cc_write_le_float(p + 6, NAN);
  CHECK(cc3_apply_tc(&c, TC_SET_VECTOR3, p, 14) == ACK_BAD_ARG &&
        !memcmp(saved, c.attitude_sp, sizeof(saved)));
  CHECK(cc3_apply_tc(&c, TC_SET_VECTOR3, p, 13) == ACK_BAD_LEN);
  p[0] = 2;
  CHECK(cc3_apply_tc(&c, TC_SET_VECTOR3, p, 14) == ACK_BAD_ARG);
  cc_write_le_float(p, 45);
  CHECK(cc3_apply_tc(&c, TC_SET_HEADING, p, 4) == ACK_OK);
  CHECK(fabsf(c.attitude_sp[0] - saved[0]) < 1e-6f &&
        fabsf(c.attitude_sp[1] - saved[1]) < 1e-6f &&
        fabsf(c.attitude_sp[2] - .785398f) < 1e-5f);
  p[0] = 1;
  p[1] = 1;
  p[2] = CC_GAIN_RATE_KP;
  cc_write_le_float(p + 3, 7);
  CHECK(cc3_apply_tc(&c, TC_SET_GAIN3, p, 7) == ACK_OK &&
        c.cfg.axis[1].rate_kp == 7 && c.cfg.axis[0].rate_kp == 5);
  CHECK(cc3_pack_tm(p, sizeof(p), &c.tlm) == 159);
  CHECK(cc3_pack_tm(p, 158, &c.tlm) == 0);
  uint8_t frame[247];
  size_t n = cc_frame_pack(frame, TM_STATE3, 9, p, 159);
  cc_parser_t parser;
  cc_parser_init(&parser);
  int found = 0;
  for (size_t i = 0; i < n; i++)
    found += cc_parser_feed(&parser, frame[i]);
  CHECK(found == 1 && parser.type == TM_STATE3 && parser.seq == 9 &&
        parser.len == 159);
  uint8_t legacy[52];
  cc_tlm_t yaw;
  cc3_get_legacy_tlm(&c, &yaw);
  cc_pack_tm_state(legacy, &yaw);
  CHECK(!memcmp(p + 3 + 104, legacy, 52));
  frame[8] ^= 1;
  cc_parser_init(&parser);
  found = 0;
  for (size_t i = 0; i < n; i++)
    found += cc_parser_feed(&parser, frame[i]);
  CHECK(found == 0);
  CHECK(cc3_apply_tc(&c, 0xef, NULL, 0) == ACK_UNKNOWN);
}
int main(void) {
  check_calibration();
  failures();
  axes();
  camera();
  protocol();
  printf("Core3 regressions: %d failures; state=%u bytes\n", fails,
         cc3_sizeof_state());
  return fails ? 1 : 0;
}
