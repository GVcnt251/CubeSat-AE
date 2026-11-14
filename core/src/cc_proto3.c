#include "cc_proto3.h"
#include "wire_le.h"
#include <math.h>
#define DEG_TO_RAD 0.017453292519943295f
#define RPM_TO_RAD 0.10471975511965977f
uint8_t cc3_apply_tc(cc3_t *c, uint8_t type, const uint8_t *p, uint8_t len) {
  if (!c || (len && !p))
    return ACK_BAD_ARG;
  if (type == TC_SET_VECTOR3) {
    if (len != 14)
      return ACK_BAD_LEN;
    if (p[0] != CC3_WIRE_VERSION)
      return ACK_BAD_ARG;
    uint32_t kind = p[1];
    if (kind != CC_CMD_WHEEL && kind != CC_CMD_RATE && kind != CC_CMD_HEADING &&
        kind != CC_CMD_AIM)
      return ACK_BAD_ARG;
    float v[3];
    for (int i = 0; i < 3; i++)
      v[i] = cc_read_le_float(p + 2 + 4 * i) *
             (kind == CC_CMD_WHEEL ? RPM_TO_RAD : DEG_TO_RAD);
    return cc3_command_vector(c, kind, v) ? ACK_OK : ACK_BAD_ARG;
  }
  if (type == TC_SET_GAIN3) {
    if (len != 7)
      return ACK_BAD_LEN;
    if (p[0] != CC3_WIRE_VERSION || p[1] > 2)
      return ACK_BAD_ARG;
    float v = cc_read_le_float(p + 3);
    switch (p[2]) {
    case CC_GAIN_W_REF_MAX:
    case CC_GAIN_A_REF_MAX:
    case CC_GAIN_R_CAM:
    case CC_GAIN_R_MAG:
      v *= DEG_TO_RAD;
      break;
    case CC_GAIN_WHEEL_LIM:
      v *= RPM_TO_RAD;
      break;
    default:
      break;
    }
    return cc3_set_gain(c, p[1], p[2], v) ? ACK_OK : ACK_BAD_ARG;
  }
  cc_cmd_t cmd;
  int r = cc_tc_to_cmd(type, p, len, &cmd);
  if (r < 0)
    return (uint8_t)-r;
  if (!r)
    return ACK_UNKNOWN;
  return cc3_command_legacy(c, &cmd) ? ACK_OK : ACK_BAD_ARG;
}
size_t cc3_pack_tm(uint8_t *out, size_t capacity, const cc3_tlm_t *tlm) {
  if (!out || !tlm || capacity < CC3_TM_SIZE)
    return 0;
  out[0] = CC3_WIRE_VERSION;
  out[1] = tlm->imu_status;
  out[2] = tlm->motor_valid_mask;
  for (int i = 0; i < 3; i++)
    cc_pack_tm_state(out + 3 + 52 * i, &tlm->axis[i]);
  return CC3_TM_SIZE;
}
