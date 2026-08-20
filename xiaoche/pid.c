#include "pid.h"
#include <math.h>

void pid_init(PID_TypeDef *pid,
              float kp, float ki, float kd,
              float sum_limit, float out_limit)
{
    pid->kp               = kp;
    pid->ki               = ki;
    pid->kd               = kd;
    pid->target           = 0.0f;
    pid->sum_error_limit_p = sum_limit;
    pid->sum_error_limit_n = -sum_limit;
    pid->output_limit_p    = out_limit;
    pid->output_limit_n    = -out_limit;
    pid->error_prev        = 0.0f;
    pid->sum_error         = 0.0f;
    pid->output            = 0.0f;
}

void pid_reset(PID_TypeDef *pid)
{
    pid->error_prev = 0.0f;
    pid->sum_error  = 0.0f;
    pid->output     = 0.0f;
}

float pid_calc(PID_TypeDef *pid, float error)
{
    float p_term, i_term, d_term, output;

    /* NaN / Inf 防护：输入非法则返回 0 */
    if (isnan(error) || isinf(error)) {
        return 0.0f;
    }

    /* 比例项 */
    p_term = pid->kp * error;

    /* 积分项：累加后限幅 */
    pid->sum_error += error;
    if (pid->sum_error > pid->sum_error_limit_p) {
        pid->sum_error = pid->sum_error_limit_p;
    } else if (pid->sum_error < pid->sum_error_limit_n) {
        pid->sum_error = pid->sum_error_limit_n;
    }
    i_term = pid->ki * pid->sum_error;

    /* 微分项 */
    d_term = pid->kd * (error - pid->error_prev);
    pid->error_prev = error;

    /* 位置式：u = Kp*e + Ki*∑e + Kd*(e[k] - e[k-1]) */
    output = p_term + i_term + d_term;

    /* NaN / Inf 二次防护 */
    if (isnan(output) || isinf(output)) {
        return 0.0f;
    }

    /* 输出限幅 */
    if (output > pid->output_limit_p) {
        output = pid->output_limit_p;
    } else if (output < pid->output_limit_n) {
        output = pid->output_limit_n;
    }

    pid->output = output;
    return output;
}
