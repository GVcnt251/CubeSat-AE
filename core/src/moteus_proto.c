#include "moteus_proto.h"
#include <string.h>
#include <math.h>

static uint8_t *pf(uint8_t *p, float v) { memcpy(p, &v, 4); return p + 4; }

static size_t fd_len(size_t n)
{
    static const uint8_t ok[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    for (unsigned i = 0; i < sizeof(ok); i++) if (ok[i] >= n) return ok[i];
    return 64;
}
static size_t finish(uint8_t *buf, uint8_t *p)
{
    size_t n = (size_t)(p - buf), m = fd_len(n);
    while (n < m) buf[n++] = 0x50;   /* NOP */
    return m;
}
static uint8_t *query(uint8_t *p)
{
    *p++ = 0x11; *p++ = 0x00;        /* int8 x1  @0x000: modo */
    *p++ = 0x1f; *p++ = 0x01;        /* float x3 @0x001: posição, velocidade, binário */
    *p++ = 0x13; *p++ = 0x0d;        /* int8 x3  @0x00d: tensão, temperatura, falha */
    return p;
}

uint32_t moteus_can_id(uint8_t src, uint8_t dst, int want_reply)
{
    return ((uint32_t)((src & 0x7f) | (want_reply ? 0x80 : 0)) << 8) | (dst & 0x7f);
}

size_t moteus_pack_velocity(uint8_t *buf, float vel, float max_torque, float accel, int q)
{
    uint8_t *p = buf;
    *p++ = 0x01; *p++ = 0x00; *p++ = 10;                 /* modo = posição */
    *p++ = 0x0c; *p++ = 6; *p++ = 0x20;                  /* 6 floats a partir de 0x020 */
    p = pf(p, NAN);                                      /* 0x020 posição = NaN -> "onde estiver" */
    p = pf(p, vel);                                      /* 0x021 velocidade (rev/s) */
    p = pf(p, 0.0f);                                     /* 0x022 binário feed-forward */
    p = pf(p, 1.0f);                                     /* 0x023 kp scale */
    p = pf(p, 1.0f);                                     /* 0x024 kd scale */
    p = pf(p, max_torque);                               /* 0x025 binário máximo */
    *p++ = 0x0d; *p++ = 0x29; p = pf(p, accel);          /* 0x029 limite de aceleração */
    if (q) p = query(p);
    return finish(buf, p);
}

size_t moteus_pack_stop(uint8_t *buf, int q)
{
    uint8_t *p = buf;
    *p++ = 0x01; *p++ = 0x00; *p++ = 0;                  /* modo = stopped */
    if (q) p = query(p);
    return finish(buf, p);
}

static int varuint(const uint8_t *b, size_t n, size_t *i, uint32_t *out)
{
    uint32_t v = 0; int sh = 0;
    for (int k = 0; k < 5; k++) {
        if (*i >= n) return -1;
        uint8_t c = b[(*i)++];
        v |= (uint32_t)(c & 0x7f) << sh; sh += 7;
        if (!(c & 0x80)) { *out = v; return 0; }
    }
    return -1;
}

/* valor de registo -> unidades SI, segundo as tabelas de mapeamento da documentação */
static float scale(uint32_t reg, int bytes, int is_float, const uint8_t *v)
{
    if (is_float) { float f; memcpy(&f, v, 4); return f; }
    int32_t x = bytes == 1 ? (int8_t)v[0] : bytes == 2 ? (int16_t)(v[0] | (v[1] << 8))
              : (int32_t)((uint32_t)v[0] | (uint32_t)v[1] << 8 | (uint32_t)v[2] << 16 | (uint32_t)v[3] << 24);
    static const float pos[3] = {0.01f, 0.0001f, 0.00001f};
    static const float vel[3] = {0.1f, 0.00025f, 0.00001f};
    static const float trq[3] = {0.5f, 0.01f, 0.001f};
    static const float vlt[3] = {0.5f, 0.1f, 0.001f};
    static const float tmp[3] = {1.0f, 0.1f, 0.001f};
    int k = bytes == 1 ? 0 : bytes == 2 ? 1 : 2;
    switch (reg) {
    case 1: return x * pos[k];
    case 2: return x * vel[k];
    case 3: return x * trq[k];
    case 0x0d: return x * vlt[k];
    case 0x0e: return x * tmp[k];
    default: return (float)x;
    }
}

int moteus_parse_reply(const uint8_t *b, size_t n, moteus_status_t *st)
{
    memset(st, 0, sizeof(*st));
    size_t i = 0;
    while (i < n) {
        uint8_t t = b[i++];
        if (t == 0x50) continue;
        if ((t & 0xf0) != 0x20) return -1;                /* só interpretamos respostas */
        int type = t & 0x0c, cnt = t & 0x03;
        int bytes = type == 0x00 ? 1 : type == 0x04 ? 2 : 4, is_float = (type == 0x0c);
        uint32_t reg, c = (uint32_t)cnt;
        if (cnt == 0 && varuint(b, n, &i, &c) < 0) return -1;
        if (varuint(b, n, &i, &reg) < 0) return -1;
        for (uint32_t k = 0; k < c; k++, reg++) {
            if (i + (size_t)bytes > n) return -1;
            float v = scale(reg, bytes, is_float, b + i);
            i += (size_t)bytes;
            switch (reg) {
            case 0x000: st->mode = (uint8_t)v; st->have_mode = 1; break;
            case 0x001: st->position_rev = v; st->have_motion = 1; break;
            case 0x002: st->velocity_rev_s = v; break;
            case 0x003: st->torque_nm = v; break;
            case 0x00d: st->voltage_v = v; st->have_aux = 1; break;
            case 0x00e: st->temp_c = v; break;
            case 0x00f: st->fault = (uint8_t)v; break;
            default: break;
            }
        }
    }
    return 0;
}
