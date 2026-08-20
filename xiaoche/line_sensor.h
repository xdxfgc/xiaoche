#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

extern uint16_t g_analog[8];
extern uint8_t  g_digital;

/* 标定数据 */
extern uint16_t g_white_ref[8];   /* 白底参考值（每通道） */
extern uint16_t g_black_ref[8];   /* 黑线参考值（每通道） */
extern uint8_t  g_calibrated;     /* 是否已完成标定 */

void linesensor_init(void);
uint8_t linesensor_update(void);
/* 诊断用：纯轮询读 ADC（不经过 DMA），返回 0xFFFF=超时 */
uint16_t read_adc_polling(void);
int16_t linesensor_get_error(void);
uint8_t linesensor_get_digital(void);

/* 标定：采白底 CALIB_SAMPLES 帧取均值 */
void linesensor_calibrate_white(void);
/* 标定：采黑线 CALIB_SAMPLES 帧取均值 */
void linesensor_calibrate_black(void);

#endif /* LINE_SENSOR_H */
