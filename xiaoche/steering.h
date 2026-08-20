#ifndef STEERING_H
#define STEERING_H

#include <stdint.h>

/* 浮点数四舍五入转 int32_t，NaN/Inf 返回 0 */
static inline int32_t round_to_i32(float val)
{
    if (val != val)       return 0;  /* NaN */
    if (val > 1e30f)      return 0;  /* +Inf */
    if (val < -1e30f)     return 0;  /* -Inf */
    if (val >= 0.0f) {
        return (int32_t)(val + 0.5f);
    } else {
        return (int32_t)(val - 0.5f);
    }
}

/* 差速转向：超限时等比例缩放（纯整数运算，M0+ 无软浮点开销） */
static inline void apply_diff_steering(int32_t raw_left, int32_t raw_right,
                                        int32_t *out_left, int32_t *out_right,
                                        int32_t pwm_max)
{
    int32_t max_abs;

    /* 找出绝对值较大的那个 */
    max_abs = raw_left;
    if (raw_left < 0) max_abs = -raw_left;
    if (raw_right > max_abs) max_abs = raw_right;
    if (-raw_right > max_abs) max_abs = -raw_right;

    if (max_abs <= pwm_max) {
        *out_left  = raw_left;
        *out_right = raw_right;
    } else {
        /* 整数等比例缩放: out = raw * pwm_max / max_abs */
        *out_left  = (int32_t)(((int64_t)raw_left  * (int64_t)pwm_max) / (int64_t)max_abs);
        *out_right = (int32_t)(((int64_t)raw_right * (int64_t)pwm_max) / (int64_t)max_abs);
    }
}

#endif /* STEERING_H */
