#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/* 毫秒计数器, SysTick ISR 中自增 */
extern volatile uint32_t g_millis;

void timebase_init(void);
uint32_t timebase_millis(void);

#endif /* TIMEBASE_H */
