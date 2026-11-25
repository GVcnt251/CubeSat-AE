/* Numeric gain rules shared by codecs and both controller interfaces.
 * Header-only so cc_proto.c never forces either controller into a build.
 */
#ifndef CC_GAIN_VALIDATION_H
#define CC_GAIN_VALIDATION_H
#include "cc_types.h"
#include <math.h>
#include <float.h>

static inline int cc_gain_value_valid(uint32_t id, float v)
{
    if (id >= CC_GAIN_COUNT || !isfinite(v) || v < 0.0f) return 0;
    if (id == CC_GAIN_USE_MAG) return v == 0.0f || v == 1.0f;
    if (id == CC_GAIN_R_CAM || id == CC_GAIN_R_MAG)
        return (double)v * v >= FLT_MIN && (double)v * v <= FLT_MAX;
    return 1;
}
#endif
