#ifndef CONFIG_H
#define CONFIG_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ==================== 传感器配置 ==================== */
#define SENSOR_COUNT               8
#define ADC_MAX_VALUE              4095
#define ADC_SAMPLES_PER_CHANNEL    8

/* 传感器物理位置（mil），用于加权偏差计算 */
extern const int16_t g_sensor_position[SENSOR_COUNT];

/* ==================== PWM 配置 ==================== */
#define PWM_PERIOD_COUNTS          1600
#define PWM_MAX                    1000
#define CONTROL_PERIOD_MS          10

/* ==================== 黑线检测阈值 ==================== */
#define LINE_PRESENT_MIN           300
#define CALIB_SAMPLES              50    /* 标定每面采样次数 */

/* ==================== CD4051 模拟开关通道选择 ==================== */
/* ch 低 3 位输出到 PB20(AD0), PB25(AD1), PB24(AD2) */
#define SWITCH_MUX_CHANNEL(ch)                                                \
    do {                                                                      \
        if ((ch) & 0x01)                                                      \
            DL_GPIO_setPins(GPIO_OUT_LINE_AD0_PORT, GPIO_OUT_LINE_AD0_PIN);   \
        else                                                                  \
            DL_GPIO_clearPins(GPIO_OUT_LINE_AD0_PORT, GPIO_OUT_LINE_AD0_PIN); \
        if ((ch) & 0x02)                                                      \
            DL_GPIO_setPins(GPIO_OUT_LINE_AD1_PORT, GPIO_OUT_LINE_AD1_PIN);   \
        else                                                                  \
            DL_GPIO_clearPins(GPIO_OUT_LINE_AD1_PORT, GPIO_OUT_LINE_AD1_PIN); \
        if ((ch) & 0x04)                                                      \
            DL_GPIO_setPins(GPIO_OUT_LINE_AD2_PORT, GPIO_OUT_LINE_AD2_PIN);   \
        else                                                                  \
            DL_GPIO_clearPins(GPIO_OUT_LINE_AD2_PORT, GPIO_OUT_LINE_AD2_PIN); \
    } while (0)

/* ==================== H 桥死区 & DMA 超时 ==================== */
#define HBRIDGE_DEADTIME_LOOPS     64
#define ADC_DMA_TIMEOUT_LOOPS      1600

/* ==================== 内联延时 ==================== */
static inline void wait_cycles(uint32_t n)
{
    while (n--) {
        __NOP();
    }
}

#endif /* CONFIG_H */
