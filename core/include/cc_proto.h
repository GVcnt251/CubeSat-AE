/*
 * cc_proto.h - protocolo TMTC entre o cubo e a estação de terra (ver docs/TMTC_ICD.md).
 *
 * Trama:  A5 5A | LEN | TYPE | SEQ | PAYLOAD[LEN] | CRC16 (LE)
 *   CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) calculado sobre LEN..PAYLOAD.
 * Little-endian. Funciona sobre UDP (1 trama por datagrama) ou série (fluxo de bytes).
 */
#ifndef CC_PROTO_H
#define CC_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include "cube_core3.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CC_SYNC0 0xA5
#define CC_SYNC1 0x5A
#define CC_MAX_PAYLOAD 240
#define CC_FRAME_OVERHEAD 7
#define CC_IMG_DS 4                      /* sub-amostragem da imagem em telemetria */
#define CC_IMG_TW (CC_IMG_W / CC_IMG_DS) /* 80 */
#define CC_IMG_TH (CC_IMG_H / CC_IMG_DS) /* 60 */
#define CC_IMG_CHUNK 200
#define CC_IMG_NCHUNKS ((CC_IMG_TW * CC_IMG_TH + CC_IMG_CHUNK - 1) / CC_IMG_CHUNK)

enum {
    TM_STATE = 0x01, TM_VISION = 0x02, TM_IMG = 0x03, TM_ACK = 0x7F,
    TC_PING = 0x80, TC_SET_MODE = 0x81, TC_SET_WHEEL = 0x82, TC_SET_RATE = 0x83,
    TC_SET_HEADING = 0x84, TC_SET_AIM = 0x85, TC_SET_GAIN = 0x86,
    TC_STREAM_CFG = 0x87, TC_RECAL = 0x88
};

enum { ACK_OK = 0, ACK_BAD_LEN = 1, ACK_UNKNOWN = 2, ACK_BAD_ARG = 3 };

uint16_t cc_crc16(const uint8_t *data, size_t n);
/* out needs len+7 bytes. Returns 0 for invalid length/pointers. */
size_t   cc_frame_pack(uint8_t *out, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len);

typedef struct {
    uint8_t  st, idx;
    uint8_t  len, type, seq;
    uint8_t  payload[CC_MAX_PAYLOAD];
    uint16_t crc;
    uint32_t n_crc_err;
} cc_parser_t;

void cc_parser_init(cc_parser_t *p);
/* 1 quando acaba de chegar uma trama válida (campos len/type/seq/payload do parser). */
int  cc_parser_feed(cc_parser_t *p, uint8_t byte);

size_t cc_pack_tm_state(uint8_t *p, const cc_tlm_t *t);
size_t cc_pack_tm_vision(uint8_t *p, const cc_vision_t *v, uint32_t proc_us);
/* chunk 0..CC_IMG_NCHUNKS-1 da imagem sub-amostrada 80x60 */
size_t cc_pack_tm_img_chunk(uint8_t *p, const uint8_t *gray, uint8_t img_id, uint8_t chunk);
size_t cc_pack_tm_ack(uint8_t *p, uint8_t type, uint8_t seq, uint8_t status);

/* Telecomando -> comando do núcleo.  1 = mapeado em *cmd; 0 = não é do núcleo; <0 = erro (-ACK_xxx) */
int cc_tc_to_cmd(uint8_t type, const uint8_t *payload, uint8_t len, cc_cmd_t *cmd);

#ifdef __cplusplus
}
#endif
#endif
