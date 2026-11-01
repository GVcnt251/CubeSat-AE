#ifndef CC_WIRE_LE_H
#define CC_WIRE_LE_H
#include <stdint.h>
#include <string.h>
#include <float.h>
#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Wire protocols require IEEE-754 binary32 floats"
#endif
typedef char cc_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];
static inline uint32_t cc_read_le32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static inline void cc_write_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline float cc_read_le_float(const uint8_t *p) {
    uint32_t u = cc_read_le32(p); float f; memcpy(&f, &u, 4); return f;
}
static inline void cc_write_le_float(uint8_t *p, float f) {
    uint32_t u; memcpy(&u, &f, 4); cc_write_le32(p, u);
}
#endif
