/* Edge checks isolated from the nominal tests. */
#include "cube_core3.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static cc3_t c;
static cc3_meas_t m;
static cc3_out_t o;
int main(void) {
  cc3_config_t g;
  cc3_config_default(&g);
  g.axis[0].rate_kp = NAN;
  assert(!cc3_init(&c, &g));
  assert(cc3_init(&c, NULL));
  m.accel[2] = 9.80665f;
  m.motor_valid_mask = 7;
  for (uint64_t t = 0; t <= 1600000; t += 5000)
    cc3_step_with_imu(&c, t, &m, 1, &o);
  assert(c.calibrated && !c.mag_reference_valid);
  float q[4];
  memcpy(q, c.q, sizeof(q));
  m.mag[1] = 44;
  m.mag_valid = 1;
  cc3_step_with_imu(&c, 1605000, &m, 1, &o);
  assert(
      !memcmp(q, c.q, sizeof(q))); /* late compass must not change the frame */
  cc_cmd_t cmd = {CC_CMD_GAIN, CC_GAIN_R_CAM, .01f, 0};
  assert(cc3_command_legacy(&c, &cmd));
  assert(fabsf(c.cfg.axis[2].r_cam - .0001f) < 1e-8f);
  cmd.id = CC_GAIN_R_MAG;
  cmd.a = .1f;
  assert(cc3_command_legacy(&c, &cmd));
  cmd.id = CC_GAIN_K_THETA;
  cmd.a = 7;
  assert(cc3_command_legacy(&c, &cmd));
  for (int i = 0; i < 3; i++)
    assert(c.cfg.axis[i].k_theta == 7);
  for (unsigned axis = 0; axis < 3; axis++) {
    assert(cc3_set_gain(&c, axis, CC_GAIN_RATE_KI, 0));
    float v[3] = {0};
    v[axis] = .5f;
    assert(cc3_command_vector(&c, CC_CMD_RATE, v));
  }
  cmd.type = CC_CMD_MODE;
  cmd.a = CC_MODE_RATE;
  assert(cc3_command_legacy(&c, &cmd));
  for (uint64_t t = 1610000; t < 1700000; t += 5000)
    cc3_step_with_imu(&c, t, &m, 1, &o);
  for (int i = 0; i < 3; i++)
    assert(c.integral[i] == 0 && isfinite(o.axis[i].wheel_cmd_rps));
  cmd.type = CC_CMD_RECAL;
  assert(cc3_command_legacy(&c, &cmd));
  for (uint64_t t = 1700000; t <= 3400000; t += 5000)
    cc3_step_with_imu(&c, t, &m, 1, &o);
  assert(c.calibrated && c.mag_reference_valid);
  /* Original scalar heading means world yaw, even with a tilted zero frame. */
  float tilted[3] = {.3f, -.4f, .5f}, yaw[3];
  cc3_quat_from_euler(tilted, c.q_zero);
  memcpy(c.q, c.q_zero, sizeof(c.q));
  memset(c.attitude_sp, 0, sizeof(c.attitude_sp));
  cmd.type = CC_CMD_HEADING;
  cmd.a = .2f;
  assert(cc3_command_legacy(&c, &cmd));
  float local[4], world[4], axis[3], temp[3];
  cc3_quat_from_euler(c.attitude_sp, local);
  /* Reconstruct R_zero * R_relative by rotating its basis columns. */
  for (int i = 0; i < 3; i++) {
    memset(axis, 0, sizeof(axis));
    axis[i] = 1;
    cc3_rotate(local, axis, temp);
    cc3_rotate(c.q_zero, temp, axis);
    tilted[2] = .7f;
    cc3_quat_from_euler(tilted, world);
    float expected[3] = {0};
    expected[i] = 1;
    cc3_rotate(world, expected, temp);
    for (int j = 0; j < 3; j++)
      assert(fabsf(axis[j] - temp[j]) < 1e-5f);
  }
  memcpy(c.q, world, sizeof(world));
  cc_tlm_t legacy;
  cc3_get_legacy_tlm(&c, &legacy);
  assert(fabsf(legacy.heading - .2f) < 1e-5f);
  cc3_quat_to_euler(c.q, yaw);
  assert(fabsf(yaw[0] - .3f) < 1e-5f);
  /* Tilt warning is informational in 3D; malformed wheel feedback is gated. */
  m.wheel_rps[1] = NAN;
  cc3_step_with_imu(&c, 3405000, &m, 1, &o);
  assert(c.tlm.axis[2].flags & CC_FLAG_TILT_HIGH);
  assert(c.tlm.motor_valid_mask == 5);
  cc3_config_default(&g);
  g.axis[2].gimbal_unload = 1;
  assert(!cc3_init(&c, &g));
  puts("Core3 configuration, late magnetometer and gain compatibility: PASS");
  return 0;
}
