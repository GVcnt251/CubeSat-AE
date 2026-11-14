/* DRAFT extension: IDs require coordination with Person 5 before deployment.
 * No changes to the original cc_proto.h, frame, CRC or legacy payloads. */
#ifndef CC_PROTO3_H
#define CC_PROTO3_H
#include "cc_proto.h"
#include "cube_core3.h"
#ifdef __cplusplus
extern "C" {
#endif
#define CC3_WIRE_VERSION 1
#define TC_SET_VECTOR3 0x89
#define TC_SET_GAIN3 0x8a
#define TM_STATE3 0x08
#define CC3_TM_SIZE 159
/* Decode and apply atomically; returns ACK status. Core commands only.
 * PING/stream configuration remain in the firmware communications layer. */
uint8_t cc3_apply_tc(cc3_t *c, uint8_t type, const uint8_t *payload,
                     uint8_t len);
/* Payload: u8 version,u8 imu_status,u8 motor_mask, three legacy 52 B states
 * XYZ. out must hold CC3_TM_SIZE bytes. Frame via cc_frame_pack(TM_STATE3,...).
 */
size_t cc3_pack_tm(uint8_t *out, size_t capacity, const cc3_tlm_t *tlm);
#ifdef __cplusplus
}
#endif
#endif
