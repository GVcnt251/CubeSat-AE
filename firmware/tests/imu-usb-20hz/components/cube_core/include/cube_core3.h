/* Standalone three-wheel core. No single-axis controller dependency.
 * Axes: body X forward, Y right, Z down. Wheels are orthogonal X/Y/Z.
 * All internal quantities are SI. q=[w,x,y,z] rotates body into world.
 * Acceleration convention: at rest/level accel=[0,0,+9.80665]. The firmware
 * must map its sensor axes/signs to this convention before calling us.
 * The caller serializes all access. No hardware or dynamic allocation here.
 */
#ifndef CUBE_CORE3_H
#define CUBE_CORE3_H
#include "cc_types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define CC3_AXES 3

typedef struct {
  cc_config_t axis[3];    /* PI/limits per wheel; camera/timing use axis[2]. */
  float accel_kp, mag_kp; /* complementary correction rates, 1/s */
  float gravity, accel_tolerance, gyro_limit, stationary_limit;
} cc3_config_t;
typedef struct {
  float gyro[3], accel[3], mag[3];
  float wheel_rps[3], wheel_torque[3], vbat_v;
  uint8_t mag_valid, motor_valid_mask; /* fresh mag; usable motors bits XYZ */
} cc3_meas_t;
typedef struct {
  cc_out_t axis[3];
} cc3_out_t;
typedef struct {
  cc_tlm_t
      axis[3]; /* Euler relative attitude; body rates; body rotation errors */
  uint8_t imu_status, motor_valid_mask;
} cc3_tlm_t;
typedef struct {
  cc3_config_t cfg;
  float q[4], q_zero[4], bias[3], rate[3];
  float wheel_sp[3], rate_sp[3], attitude_sp[3], aim[3];
  float integral[3], ref[3], err[3];
  float target_world[3], hist_q[CC_HIST][4];
  uint64_t hist_t[CC_HIST];
  unsigned head, count;
  uint64_t last_call, last_valid, calib_start, last_cam, last_capture,
      track_entry;
  int call_seen, imu_seen, missing, blocked, recal_requested, calibrated,
      calibrated_once;
  int cam_seen, target_valid, search_known, rejects, mag_ok;
  int mag_reference_valid; /* No late compass-frame change: recalibrate if
                              absent at startup. */
  cc_mode_t mode;
  cc_trk_t trk;
  float search_center, search_dir;
  double gyro_sum[3], gyro_sq[3], accel_sum[3], mag_sum[3];
  unsigned calib_count, mag_count;
  cc3_tlm_t tlm;
} cc3_t;
/* Public numeric gain validator; same rules as the shared TMTC decoder. */
int cc3_gain_valid(uint32_t id, float value);
void cc3_config_default(cc3_config_t *cfg);
/* 0 on invalid config/pointer; otherwise 1. State is caller-owned. */
int cc3_init(cc3_t *c, const cc3_config_t *cfg);
/* MODE and RECAL are global. Legacy RATE/WHEEL/AIM affect Z only; HEADING sets
 * world yaw relative to initial yaw, preserving desired world roll/pitch.
 * Legacy gains affect all axes (sensor/timing gains are shared).
 * Return 1 applied, 0 rejected. A mode command during CALIB is rejected.
 */
int cc3_command_legacy(cc3_t *c, const cc_cmd_t *cmd);
/* kind=WHEEL/RATE/HEADING/AIM. HEADING: relative Euler ZYX [roll,pitch,yaw].
 * AIM: [absolute-world roll reference, vertical bearing down+, horizontal
 * right+]. The point alone cannot observe roll. Roll uses IMU/magnetic
 * reference. Setpoint changes do not change mode. Return 1 applied, 0 rejected
 * atomically. */
int cc3_command_vector(cc3_t *c, uint32_t kind, const float values[3]);
int cc3_set_gain(cc3_t *c, unsigned axis, uint32_t id, float value);
cc_imu_status_t cc3_step_with_imu(cc3_t *c, uint64_t now_us,
                                  const cc3_meas_t *m, int imu_valid,
                                  cc3_out_t *out);
/* A single fresh camera detection updates a world-space ray for both yaw/pitch.
 * Camera optical axis +X, image right +Y, down +Z. Timestamp shares IMU clock.
 */
int cc3_on_detection(cc3_t *c, uint64_t capture_us, uint64_t now_us,
                     const cc_det_t *d, cc_vision_t *out);
void cc3_get_tlm(const cc3_t *c, cc3_tlm_t *out);
/* Reuse existing TM_STATE packer on this yaw-only projection. */
void cc3_get_legacy_tlm(const cc3_t *c, cc_tlm_t *out);
void cc3_get_attitude(const cc3_t *c, float q[4]);
uint32_t cc3_sizeof_state(void);
uint32_t cc3_sizeof_meas(void);
uint32_t cc3_sizeof_out(void);
uint32_t cc3_sizeof_tlm(void);
/* Reference frame/rotation helpers, also used by host verification. */
void cc3_quat_from_euler(const float rpy[3], float q[4]);
void cc3_quat_to_euler(const float q[4], float rpy[3]);
void cc3_rotate(const float q[4], const float v[3], float out[3]);
#ifdef __cplusplus
}
#endif
#endif
