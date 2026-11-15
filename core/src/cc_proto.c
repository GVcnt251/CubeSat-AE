#include "cc_proto.h"
#include <string.h>
#include <math.h>
#include "wire_le.h"

#define RAD2DEG 57.29577951308232f
#define DEG2RAD 0.017453292519943295f
#define RPS2RPM 9.549296585513720f      /* rad/s -> rpm */
#define RPM2RPS 0.10471975511965977f

uint16_t cc_crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t cc_frame_pack(uint8_t *o, uint8_t type, uint8_t seq, const uint8_t *pl, uint8_t len)
{
    if (!o || len > CC_MAX_PAYLOAD || (len && !pl)) return 0;
    o[0] = CC_SYNC0; o[1] = CC_SYNC1; o[2] = len; o[3] = type; o[4] = seq;
    if (len) memcpy(o + 5, pl, len);
    uint16_t crc = cc_crc16(o + 2, (size_t)len + 3);
    o[5 + len] = (uint8_t)(crc & 0xFF); o[6 + len] = (uint8_t)(crc >> 8);
    return (size_t)len + CC_FRAME_OVERHEAD;
}

void cc_parser_init(cc_parser_t *p) { memset(p, 0, sizeof(*p)); }

int cc_parser_feed(cc_parser_t *p, uint8_t b)
{
    switch (p->st) {
    case 0: if (b == CC_SYNC0) p->st = 1; break;
    case 1: p->st = (b == CC_SYNC1) ? 2 : (b == CC_SYNC0 ? 1 : 0); break;
    case 2: if (b > CC_MAX_PAYLOAD) { p->st = 0; break; } p->len = b; p->st = 3; break;
    case 3: p->type = b; p->st = 4; break;
    case 4: p->seq = b; p->idx = 0; p->st = p->len ? 5 : 6; break;
    case 5: p->payload[p->idx++] = b; if (p->idx >= p->len) p->st = 6; break;
    case 6: p->crc = b; p->st = 7; break;
    case 7: {
        p->crc |= (uint16_t)b << 8;
        uint8_t hdr[3] = { p->len, p->type, p->seq };
        uint16_t c = 0xFFFF;
        /* CRC incremental sobre cabeçalho + payload */
        uint8_t tmp[CC_MAX_PAYLOAD + 3];
        memcpy(tmp, hdr, 3); memcpy(tmp + 3, p->payload, p->len);
        c = cc_crc16(tmp, (size_t)p->len + 3);
        p->st = 0;
        if (c == p->crc) return 1;
        p->n_crc_err++;
        break;
    }
    }
    return 0;
}

static void put_u32(uint8_t **p, uint32_t v) { cc_write_le32(*p, v); *p += 4; }
static void put_u16(uint8_t **p, uint16_t v) { (*p)[0] = (uint8_t)v; (*p)[1] = (uint8_t)(v >> 8); *p += 2; }
static void put_f32(uint8_t **p, float v)    { cc_write_le_float(*p, v); *p += 4; }
static void put_u8(uint8_t **p, uint8_t v)   { *(*p)++ = v; }
static float get_f32(const uint8_t *p)       { return cc_read_le_float(p); }

size_t cc_pack_tm_state(uint8_t *p0, const cc_tlm_t *t)
{
    uint8_t *p = p0;
    put_u32(&p, t->t_ms); put_u8(&p, t->mode); put_u8(&p, t->trk); put_u16(&p, t->flags);
    put_f32(&p, t->heading * RAD2DEG);
    put_f32(&p, t->rate * RAD2DEG);
    put_f32(&p, t->bias * RAD2DEG);
    put_f32(&p, t->err * RAD2DEG);
    put_f32(&p, t->rate_ref * RAD2DEG);
    put_f32(&p, t->wheel_rps * RPS2RPM);
    put_f32(&p, t->wheel_cmd_rps * RPS2RPM);
    put_f32(&p, t->torque_nm);
    put_f32(&p, t->vbat_v);
    put_f32(&p, t->target_err * RAD2DEG);
    put_f32(&p, t->cam_age_s);
    return (size_t)(p - p0);
}

size_t cc_pack_tm_vision(uint8_t *p0, const cc_vision_t *v, uint32_t proc_us)
{
    uint8_t *p = p0;
    put_u32(&p, v->t_capture_ms);
    put_u8(&p, v->found); put_u8(&p, v->accepted); put_u8(&p, v->n_cand); put_u8(&p, 0);
    put_f32(&p, v->x); put_f32(&p, v->y);
    put_f32(&p, v->err_x * RAD2DEG); put_f32(&p, v->err_y * RAD2DEG);
    put_f32(&p, v->contrast);
    put_u32(&p, v->area); put_u32(&p, proc_us);
    return (size_t)(p - p0);
}

size_t cc_pack_tm_img_chunk(uint8_t *p0, const uint8_t *g, uint8_t img_id, uint8_t chunk)
{
    if (!p0 || !g || chunk >= CC_IMG_NCHUNKS) return 0;
    uint8_t *p = p0;
    put_u8(&p, img_id); put_u8(&p, chunk); put_u8(&p, CC_IMG_NCHUNKS);
    put_u8(&p, CC_IMG_TW); put_u8(&p, CC_IMG_TH);
    int first = chunk * CC_IMG_CHUNK, total = CC_IMG_TW * CC_IMG_TH;
    for (int i = first; i < first + CC_IMG_CHUNK && i < total; i++) {
        int r = i / CC_IMG_TW, c = i % CC_IMG_TW;
        unsigned s = 0;
        for (int dy = 0; dy < CC_IMG_DS; dy++)
            for (int dx = 0; dx < CC_IMG_DS; dx++)
                s += g[(r * CC_IMG_DS + dy) * CC_IMG_W + c * CC_IMG_DS + dx];
        *p++ = (uint8_t)(s / (CC_IMG_DS * CC_IMG_DS));
    }
    return (size_t)(p - p0);
}

size_t cc_pack_tm_ack(uint8_t *p, uint8_t type, uint8_t seq, uint8_t status)
{
    p[0] = type; p[1] = seq; p[2] = status; return 3;
}

int cc_tc_to_cmd(uint8_t type, const uint8_t *pl, uint8_t len, cc_cmd_t *cmd)
{
    if (!cmd) return -ACK_BAD_ARG;
    memset(cmd, 0, sizeof(*cmd));
    if (len && !pl) return -ACK_BAD_ARG;
    switch (type) {
    case TC_SET_MODE:
        if (len != 1) return -ACK_BAD_LEN;
        if (pl[0] < CC_MODE_IDLE || pl[0] > CC_MODE_TRACK) return -ACK_BAD_ARG;
        cmd->type = CC_CMD_MODE; cmd->a = (float)pl[0]; return 1;
    case TC_SET_WHEEL:   /* rpm */
        if (len != 4) return -ACK_BAD_LEN;
        if (!isfinite(get_f32(pl))) return -ACK_BAD_ARG;
        cmd->type = CC_CMD_WHEEL; cmd->a = get_f32(pl) * RPM2RPS; return 1;
    case TC_SET_RATE:    /* deg/s */
        if (len != 4) return -ACK_BAD_LEN;
        if (!isfinite(get_f32(pl))) return -ACK_BAD_ARG;
        cmd->type = CC_CMD_RATE; cmd->a = get_f32(pl) * DEG2RAD; return 1;
    case TC_SET_HEADING: /* deg */
        if (len != 4) return -ACK_BAD_LEN;
        if (!isfinite(get_f32(pl))) return -ACK_BAD_ARG;
        cmd->type = CC_CMD_HEADING; cmd->a = get_f32(pl) * DEG2RAD; return 1;
    case TC_SET_AIM:     /* deg */
        if (len != 4) return -ACK_BAD_LEN;
        if (!isfinite(get_f32(pl))) return -ACK_BAD_ARG;
        cmd->type = CC_CMD_AIM; cmd->a = get_f32(pl) * DEG2RAD; return 1;
    case TC_SET_GAIN: {  /* u8 id + f32 valor (unidades do GS; ver ICD) */
        if (len != 5) return -ACK_BAD_LEN;
        if (pl[0] >= CC_GAIN_COUNT) return -ACK_BAD_ARG;
        float v = get_f32(pl + 1);
        switch (pl[0]) {
        case CC_GAIN_W_REF_MAX: case CC_GAIN_A_REF_MAX: case CC_GAIN_R_CAM: case CC_GAIN_R_MAG: v *= DEG2RAD; break;
        case CC_GAIN_WHEEL_LIM: v *= RPM2RPS; break;
        default: break;
        }
        if (!cc3_gain_valid(pl[0], v)) return -ACK_BAD_ARG;
        cmd->type = CC_CMD_GAIN; cmd->id = pl[0]; cmd->a = v; return 1;
    }
    case TC_PING:
        return len == 0 ? 0 : -ACK_BAD_LEN;
    case TC_STREAM_CFG:
        return len == 3 ? 0 : -ACK_BAD_LEN;
    case TC_RECAL:
        if (len != 0) return -ACK_BAD_LEN;
        cmd->type = CC_CMD_RECAL; return 1;
    default:
        return 0;
    }
}
