#include "motor.h"
#include "config.h"

/* 记录当前方向状态，用于判断是否需要死区切换 */
static int32_t g_left_dir  = 0;  /* 0=停止, 1=前进, -1=后退 */
static int32_t g_right_dir = 0;

void motor_init(void)
{
    /* PWM 和 GPIO 已由 SYSCFG_DL_init() 完成初始化 */

    /* PWM 初始占空比置 0，停止状态 */
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_PERIOD_COUNTS, GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_PERIOD_COUNTS, GPIO_MOTOR_PWM_C1_IDX);

    /* 方向引脚全部拉低 */
    DL_GPIO_clearPins(GPIO_OUT_AIN1_PORT, GPIO_OUT_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_OUT_AIN2_PORT, GPIO_OUT_AIN2_PIN);
    DL_GPIO_clearPins(GPIO_OUT_BIN1_PORT, GPIO_OUT_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_OUT_BIN2_PORT, GPIO_OUT_BIN2_PIN);

    /* 启动 PWM 计数器（占空比为 0，电机不转） */
    DL_TimerA_startCounter(MOTOR_PWM_INST);

    g_left_dir  = 0;
    g_right_dir = 0;
}

/* 设置单个电机的方向和 PWM 比较值 */
static void motor_set_one(uint16_t speed, int32_t new_dir,
                          GPIO_Regs *in1_port, uint32_t in1_pin,
                          GPIO_Regs *in2_port, uint32_t in2_pin,
                          uint8_t cc_idx,
                          int32_t *prev_dir)
{
    uint32_t cmp_val;

    if (speed == 0 || new_dir == 0) {
        /* 停止：PWM 写 0，方向 GPIO 全拉低 */
        DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST,
            PWM_PERIOD_COUNTS, cc_idx);
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
        *prev_dir = 0;
        return;
    }

    /* 限幅 */
    if (speed > PWM_MAX) speed = (uint16_t)PWM_MAX;
    cmp_val = PWM_PERIOD_COUNTS - speed;  /* 低电平有效 PWM */

    if (new_dir != *prev_dir) {
        /* 方向翻转：先停 PWM → 死区 → 翻转 GPIO → 恢复 PWM */
        DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST,
            PWM_PERIOD_COUNTS, cc_idx);  /* 先拉高（占空比 0） */

        {
            uint32_t n;
            for (n = 0; n < HBRIDGE_DEADTIME_LOOPS; n++) {
                __NOP();
            }
        }

        /* 翻转方向 GPIO */
        if (new_dir > 0) {
            DL_GPIO_setPins(in1_port, in1_pin);
            DL_GPIO_clearPins(in2_port, in2_pin);
        } else {
            DL_GPIO_clearPins(in1_port, in1_pin);
            DL_GPIO_setPins(in2_port, in2_pin);
        }
    }

    /* 更新 PWM 比较值 */
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST, cmp_val, cc_idx);
    *prev_dir = new_dir;
}

void motor_set_speed(int32_t left, int32_t right)
{
    int32_t left_dir, right_dir;
    uint16_t left_abs, right_abs;

    /* 分解符号和幅度 */
    if (left > 0) {
        left_dir = 1;
        left_abs = (uint16_t)left;
    } else if (left < 0) {
        left_dir = -1;
        left_abs = (uint16_t)(-left);
    } else {
        left_dir = 0;
        left_abs = 0;
    }

    if (right > 0) {
        right_dir = 1;
        right_abs = (uint16_t)right;
    } else if (right < 0) {
        right_dir = -1;
        right_abs = (uint16_t)(-right);
    } else {
        right_dir = 0;
        right_abs = 0;
    }

    /* 左电机：TIMA1_CCP0, 前进 PA13 高 / PA12 低 */
    motor_set_one(left_abs, left_dir,
                  GPIO_OUT_AIN1_PORT, GPIO_OUT_AIN1_PIN,
                  GPIO_OUT_AIN2_PORT, GPIO_OUT_AIN2_PIN,
                  GPIO_MOTOR_PWM_C0_IDX, &g_left_dir);

    /* 右电机：TIMA1_CCP1, 前进 PB0 高 / PB8 低 */
    motor_set_one(right_abs, right_dir,
                  GPIO_OUT_BIN1_PORT, GPIO_OUT_BIN1_PIN,
                  GPIO_OUT_BIN2_PORT, GPIO_OUT_BIN2_PIN,
                  GPIO_MOTOR_PWM_C1_IDX, &g_right_dir);
}

void motor_stop(void)
{
    /* PWM 比较置最大值（占空比 0） */
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_PERIOD_COUNTS, GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_PERIOD_COUNTS, GPIO_MOTOR_PWM_C1_IDX);

    /* 方向引脚全拉低 */
    DL_GPIO_clearPins(GPIO_OUT_AIN1_PORT, GPIO_OUT_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_OUT_AIN2_PORT, GPIO_OUT_AIN2_PIN);
    DL_GPIO_clearPins(GPIO_OUT_BIN1_PORT, GPIO_OUT_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_OUT_BIN2_PORT, GPIO_OUT_BIN2_PIN);

    g_left_dir  = 0;
    g_right_dir = 0;
}
