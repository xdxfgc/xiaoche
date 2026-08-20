#include "config.h"
#include "timebase.h"
#include "pid.h"
#include "steering.h"
#include "motor.h"
#include "line_sensor.h"

/* 标定开关：首次调试时设为 0（先用 g_dbg_adc 确认传感器读数正常），正常使用时设为 1 */
#define ENABLE_CALIBRATION  0

/* ==================== PID 控制器 ==================== */
PID_TypeDef g_pid;

/* ==================== 参数设置 ==================== */
static float g_base_speed = 250.0f;

/* ==================== 调试变量（CCS/J-Link Watch 窗口可观察） ==================== */
volatile uint16_t g_dbg_adc[SENSOR_COUNT];  /* 8 路 ADC 原始值（镜像） */
volatile uint8_t  g_dbg_digital;            /* 8 路二值化位图 */
volatile int16_t  g_dbg_error;              /* 位置偏差 */
volatile int32_t  g_dbg_left_cmd;           /* 左轮 PWM */
volatile int32_t  g_dbg_right_cmd;          /* 右轮 PWM */
volatile uint8_t  g_dbg_line_found;         /* 是否检测到线 */

/* ==================== 主函数 ==================== */
int main(void)
{
    uint32_t next_ms;

    /* 1. 硬件初始化 */
    SYSCFG_DL_init();

    /* 使能 DMA 中断 */
    NVIC_EnableIRQ(DMA_INT_IRQn);

    /* 启动 1ms 时基 */
    timebase_init();

    /* PID 初始化: Kp=0.16, Ki=0, Kd=0（纯P控制）, 积分限幅 ±100000, 输出限幅 ±1000 */
    pid_init(&g_pid, 0.16f, 0.0f, 0.0f, 100000.0f, 1000.0f);

    /* 电机和传感器初始化 */
    motor_init();
    motor_stop();
    linesensor_init();

    /* 2. 延时 2s，等待稳定 */
 

#if ENABLE_CALIBRATION
    /* 3. 标定流程
     *    用户先将小车放在白底上开机 → 2s 后自动采白底
     *    → 等待 5s（用户将车移到黑线上）
     *    → 自动采黑线 → 标定完成
     */
    linesensor_calibrate_white();
    {
        uint32_t start = timebase_millis();
        while (timebase_millis() - start < 5000) {
            /* 空轮询，等待用户放置到黑线上 */
        }
    }
    linesensor_calibrate_black();

    /* 延时 1s 后开始运行 */
    {
        uint32_t start = timebase_millis();
        while (timebase_millis() - start < 1000) {
            /* 空轮询 */
        }
    }
#else
    /* 标定未启用：使用硬阈值 1500 + 原始暗度（见 line_sensor.c 未标定分支） */
    /* 正常使用前请将 ENABLE_CALIBRATION 改为 1 */
#endif

    /* 4. 主控制循环（10ms 周期） */
    next_ms = timebase_millis() + CONTROL_PERIOD_MS;

    /*
     * ===== 诊断模式：测 ADC 是否正常工作 =====
     * 在下面这行设断点，每次命中断点看 g_dbg_poll_val。
     * 0xFFFF = ADC 没跑起来，非零值 = ADC 正常。
     * 测完后恢复下面的正常主循环。
     */
    volatile uint16_t g_dbg_poll_val;
    while (1) {
        g_dbg_poll_val = read_adc_polling();
        g_dbg_adc[0] = g_dbg_poll_val;
        wait_cycles(320000);   /* ~10ms，不会卡死 */
    }
    
    
    
}
