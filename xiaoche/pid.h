#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <float.h>

/* 位置式 PID 控制器结构体 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float sum_error_limit_p;   /* 积分正限幅 */
    float sum_error_limit_n;   /* 积分负限幅 */
    float output_limit_p;      /* 输出正限幅 */
    float output_limit_n;      /* 输出负限幅 */
    float error_prev;          /* 上一次误差 */
    float sum_error;           /* 积分累加 */
    float output;              /* 当前输出 */
} PID_TypeDef;

void pid_init(PID_TypeDef *pid,
              float kp, float ki, float kd,
              float sum_limit, float out_limit);

void pid_reset(PID_TypeDef *pid);

float pid_calc(PID_TypeDef *pid, float error);

#endif /* PID_H */
