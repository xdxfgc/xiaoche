#include "config.h"
#include "timebase.h"

volatile uint32_t g_millis = 0;

void timebase_init(void)
{
    /* 配置 SysTick 产生 1ms 中断, CPUCLK_FREQ=32MHz */
    SysTick_Config(CPUCLK_FREQ / 1000);

    /*
     * MSPM0 上 SysTick 可能被启动代码复位，手动确保使能。
     * CTRL 寄存器: bit2=CLKSOURCE(1=CPUCLK), bit1=TICKINT, bit0=ENABLE
     */
    SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk
                    | SysTick_CTRL_TICKINT_Msk
                    | SysTick_CTRL_ENABLE_Msk);
}

uint32_t timebase_millis(void)
{
    return g_millis;
}

/* SysTick 中断服务函数 */
void SysTick_Handler(void)
{
    g_millis++;
}
