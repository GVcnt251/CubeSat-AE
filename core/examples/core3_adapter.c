/* Integration example only: Person 1 owns scheduling, locks, CAN IDs and signs.
 * Compile as an object to check the API; not an independent firmware
 * application. All functions below must be called with the same external core
 * mutex held.
 */
#include "cc_proto3.h"
static cc3_t core3;
int example_core3_init(void) { return cc3_init(&core3, 0); }
cc_imu_status_t example_core3_cycle(uint64_t t, const cc3_meas_t *fresh,
                                    int imu_valid, cc3_out_t *out) {
  /* fresh: all vectors in BODY XYZ. motor_valid_mask must reflect fresh
   * fault-free replies, not merely CAN transmission success.
   * Firmware sends out->axis[i] to its configured moteus ID/sign for axis i. */
  return cc3_step_with_imu(&core3, t, fresh, imu_valid, out);
}
uint8_t example_core3_tc(uint8_t type, const uint8_t *p, uint8_t len,
                         int draft_enabled) {
  /* PING and STREAM_CFG are handled by the existing comms task, before this.
   * Default draft_enabled=0 until Person 5 accepts the extension. */
  if (!draft_enabled && (type == TC_SET_VECTOR3 || type == TC_SET_GAIN3))
    return ACK_UNKNOWN;
  return cc3_apply_tc(&core3, type, p, len);
}
size_t example_core3_legacy_state(uint8_t payload[52]) {
  cc_tlm_t yaw;
  cc3_get_legacy_tlm(&core3, &yaw);
  return cc_pack_tm_state(payload, &yaw);
}
size_t example_core3_extended_state(uint8_t *payload, size_t capacity) {
  cc3_tlm_t all;
  cc3_get_tlm(&core3, &all);
  return cc3_pack_tm(payload, capacity, &all);
}
int example_core3_detection(uint64_t capture, uint64_t now, const cc_det_t *d,
                            cc_vision_t *v) {
  /* cc_detect runs separately using one caller-owned cc_det_ws_t, outside lock.
   */
  return cc3_on_detection(&core3, capture, now, d, v);
}
