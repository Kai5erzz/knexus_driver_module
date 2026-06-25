#ifndef KNX_MATH_H
#define KNX_MATH_H

#include <stdint.h>

/* Clamp integer to [min, max] */
static inline int32_t knx_clamp_i32(int32_t val, int32_t min, int32_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* Clamp float to [min, max] */
static inline float knx_clamp_f(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* Absolute value for int32 */
static inline int32_t knx_abs_i32(int32_t val)
{
    return (val < 0) ? -val : val;
}

/* Absolute value for float */
static inline float knx_abs_f(float val)
{
    return (val < 0.0f) ? -val : val;
}

/* Linear interpolation: a + t*(b-a), t in [0,1] */
static inline float knx_lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

/* Deadzone: returns 0 if |val| < threshold, else val */
static inline float knx_deadzone(float val, float threshold)
{
    return (knx_abs_f(val) < threshold) ? 0.0f : val;
}

#endif /* KNX_MATH_H */
