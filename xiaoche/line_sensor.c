#include "line_sensor.h"
#include "config.h"

/* ==================== 全局状态变量 ==================== */
uint16_t g_analog[SENSOR_COUNT];    /* 8 路传感器 ADC 均值 */
uint8_t  g_digital;                 /* 二值化位图，bit i=1 表示检测到黑线 */
int16_t  g_error;                   /* 加权位置偏差 */
uint8_t  g_line_found;              /* 是否检测到线 */

/* DMA 缓冲区 & 同步标志 */
volatile uint16_t g_adc_dma_buf[ADC_SAMPLES_PER_CHANNEL];
volatile uint8_t  g_dma_done;

/* 传感器位置常量（mil） */
const int16_t g_sensor_position[SENSOR_COUNT] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

/* ==================== 标定数据 ==================== */
uint16_t g_white_ref[SENSOR_COUNT];   /* 白底参考值 */
uint16_t g_black_ref[SENSOR_COUNT];   /* 黑线参考值 */
uint8_t  g_calibrated;                /* 标定完成标志 */

/* ADC12 MEM0 结果寄存器地址 */
#define ADC12_MEMRES_OFFSET  0x0080U
#define ADC12_MEM0_ADDR      ((uint32_t)LINE_ADC_INST + ADC12_MEMRES_OFFSET \
                               + (LINE_ADC_ADCMEM_0 * 4U))

/* ==================== DMA 中断服务函数 ==================== */
void DMA_IRQHandler(void)
{
    /* 判断是否为通道 0 中断 */
    if (DL_DMA_getRawInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0)) {
        /* 停止 DMA 通道和 ADC 转换 */
        DL_DMA_disableChannel(DMA, LINE_DMA_CHAN_ID);
        DL_ADC12_stopConversion(LINE_ADC_INST);

        /* 清除中断标志 */
        DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);

        /* 数据同步屏障 */
        __DSB();

        /* 通知主循环 DMA 完成 */
        g_dma_done = 1;
    }
}

/* ==================== 初始化 ==================== */
void linesensor_init(void)
{
    uint8_t i;

    /* 清空内部状态 */
    g_digital    = 0;
    g_error      = 0;
    g_line_found = 0;
    g_dma_done   = 0;
    g_calibrated = 0;

    for (i = 0; i < SENSOR_COUNT; i++) {
        g_analog[i]    = 0;
        g_white_ref[i] = 0;
        g_black_ref[i] = 0;
    }
    for (i = 0; i < ADC_SAMPLES_PER_CHANNEL; i++) {
        g_adc_dma_buf[i] = 0;
    }
}

/* ==================== 单通道 ADC DMA 突发采样 ==================== */
/*
 * DMA 通道配置由 SysConfig 在 SYSCFG_DL_DMA_init() 中一次性完成。
 * 此处只需更新传输参数，不再重复 initChannel，避免破坏触发链。
 *   1. 停 ADC → 禁 DMA → 清中断标志 → 复位 NVIC
 *   2. 更新源/目标地址和传输长度
 *   3. 切换 CD4051 → 等待稳定
 *   4. 使能 DMA 中断 → 使能通道 → 启动 ADC
 *   5. 轮询等待完成 → 超时/成功求均值
 */
static uint8_t read_adc_burst_dma(uint8_t ch, uint16_t *result)
{
    uint32_t sum;
    uint32_t timeout;
    uint8_t i;

    /* 1. 停 ADC，禁 DMA，清所有挂起标志 */
    DL_ADC12_stopConversion(LINE_ADC_INST);
    DL_DMA_disableChannel(DMA, LINE_DMA_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    g_dma_done = 0;

    /* 清除 ADC MEM0 中断标志 */
    DL_ADC12_clearInterruptStatus(LINE_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    /* 2. 更新本次传输参数（不重新 initChannel，保留 SysConfig 的触发配置） */
    DL_DMA_setSrcAddr(DMA, LINE_DMA_CHAN_ID, ADC12_MEM0_ADDR);
    DL_DMA_setDestAddr(DMA, LINE_DMA_CHAN_ID, (uint32_t)g_adc_dma_buf);
    DL_DMA_setTransferSize(DMA, LINE_DMA_CHAN_ID, ADC_SAMPLES_PER_CHANNEL);

    /* 3. 切换 CD4051 通道 */
    SWITCH_MUX_CHANNEL(ch);
    wait_cycles(640);

    /* 4. 使能 DMA 中断 → 使能 DMA 通道 → 启动 ADC */
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableChannel(DMA, LINE_DMA_CHAN_ID);
    DL_ADC12_startConversion(LINE_ADC_INST);

    /* 5. 轮询等待 DMA 完成 */
    timeout = ADC_DMA_TIMEOUT_LOOPS;
    while (!g_dma_done && timeout > 0) {
        timeout--;
        __NOP();
    }

    /* 超时处理 */
    if (timeout == 0) {
        DL_ADC12_stopConversion(LINE_ADC_INST);
        DL_DMA_disableChannel(DMA, LINE_DMA_CHAN_ID);
        return 0;
    }

    /* 7. 8 样本求均值 */
    sum = 0;
    for (i = 0; i < ADC_SAMPLES_PER_CHANNEL; i++) {
        sum += g_adc_dma_buf[i];
    }
    *result = (uint16_t)(sum / ADC_SAMPLES_PER_CHANNEL);

    return 1;
}

/* ==================== 诊断：纯轮询 ADC 读取（不依赖 DMA） ==================== */
/*
 * 用于排查 ADC 硬件是否正常。
 * 不碰 DMA，靠轮询 ADC IFG 标志等转换完成。
 * 返回 MEM0 寄存器直接读取值，失败返回 0xFFFF。
 */
uint16_t read_adc_polling(void)
{
    uint32_t timeout;

    /* 确保 ADC 停止，清空中断标志 */
    DL_ADC12_stopConversion(LINE_ADC_INST);
    DL_ADC12_clearInterruptStatus(LINE_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    /* 软件触发一次转换 */
    DL_ADC12_startConversion(LINE_ADC_INST);

    /* 轮询等 IFG（MEM0 结果就绪） */
    timeout = 100000;
    while (!(DL_ADC12_getRawInterruptStatus(LINE_ADC_INST,
               DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED))) {
        if (--timeout == 0) {
            DL_ADC12_stopConversion(LINE_ADC_INST);
            return 0xFFFF;  /* 超时 → ADC 根本没跑 */
        }
    }

    DL_ADC12_stopConversion(LINE_ADC_INST);

    return DL_ADC12_getMemResult(LINE_ADC_INST, LINE_ADC_ADCMEM_0);
}

/* ==================== 标定：采集参考面 ==================== */
/*
 * 采 CALIB_SAMPLES 帧，每帧读取全部 8 通道，累加后取均值存入 ref 数组。
 */
static void calibrate_surface(uint16_t *ref)
{
    uint16_t frame;
    uint8_t  ch;
    uint32_t sum[SENSOR_COUNT];
    uint16_t dummy;

    /* 清零累加器 */
    for (ch = 0; ch < SENSOR_COUNT; ch++) {
        sum[ch] = 0;
    }

    for (frame = 0; frame < CALIB_SAMPLES; frame++) {
        for (ch = 0; ch < SENSOR_COUNT; ch++) {
            if (read_adc_burst_dma(ch, &dummy)) {
                sum[ch] += (uint32_t)dummy;
            }
            /* 失败时沿用上次值，不计入累加 */
        }
    }

    for (ch = 0; ch < SENSOR_COUNT; ch++) {
        ref[ch] = (uint16_t)(sum[ch] / (uint32_t)CALIB_SAMPLES);
    }
}

void linesensor_calibrate_white(void)
{
    calibrate_surface(g_white_ref);
}

void linesensor_calibrate_black(void)
{
    calibrate_surface(g_black_ref);
    g_calibrated = 1;  /* 黑白都采完才算标定完成 */
}

/* ==================== 8 路传感器批量更新 ==================== */
/*
 * 依次读取 8 个通道的 ADC 值，二值化后加权计算位置偏差。
 * 标定后使用归一化黑度值（每通道独立阈值），消除探头个体差异。
 * 返回 1 表示检测到线，0 表示丢线。
 */
uint8_t linesensor_update(void)
{
    uint8_t  ch;
    uint32_t sum_dark;
    int32_t  weighted_sum;
    uint8_t  digital = 0;

    /* 循环读取 8 路模拟值 */
    for (ch = 0; ch < SENSOR_COUNT; ch++) {
        if (!read_adc_burst_dma(ch, &g_analog[ch])) {
            g_line_found = 0;
            return 0;
        }
    }

    /* 二值化 & 加权计算 */
    sum_dark     = 0;
    weighted_sum = 0;

    if (g_calibrated) {
        /* --- 标定模式：归一化黑度 --- */
        uint32_t range;
        for (ch = 0; ch < SENSOR_COUNT; ch++) {
            /* 二值化：analog 低于 (white + black)/2 判定为黑线 */
            if (g_analog[ch] < (uint32_t)(g_white_ref[ch] + g_black_ref[ch]) / 2U) {
                digital |= (1U << ch);
            }
            /* 归一化：0=黑 → 1000=白 */
            range = (uint32_t)g_white_ref[ch] - (uint32_t)g_black_ref[ch];
            if (range < 10U) {
                range = 10U;  /* 防止除零 */
            }
            {
                /* dark = 1000 - (analog - black) * 1000 / range */
                int32_t norm = (int32_t)(g_analog[ch] - g_black_ref[ch]) * 1000
                               / (int32_t)range;
                if (norm < 0)   norm = 0;
                if (norm > 1000) norm = 1000;
                {
                    int32_t dark = 1000 - norm;
                    sum_dark     += (uint32_t)dark;
                    weighted_sum += dark * (int32_t)g_sensor_position[ch];
                }
            }
        }
    } else {
        /* --- 未标定模式：原始暗度 --- */
        for (ch = 0; ch < SENSOR_COUNT; ch++) {
            if (g_analog[ch] < 1500) {   /* 硬阈值 1500 */
                digital |= (1U << ch);
            }
            {
                uint32_t dark = (uint32_t)(ADC_MAX_VALUE - g_analog[ch]);
                sum_dark     += dark;
                weighted_sum += (int32_t)dark * (int32_t)g_sensor_position[ch];
            }
        }
    }

    g_digital = digital;

    /* 判断是否丢线 */
    if (sum_dark < 10) {
        g_line_found = 0;
        return 0;
    }

    /* 加权平均偏差 = Σ(dark × position) / Σ(dark) */
    g_error      = (int16_t)(weighted_sum / (int32_t)sum_dark);
    g_line_found = 1;
    return 1;
}

/* ==================== 访问器 ==================== */
int16_t linesensor_get_error(void)
{
    return g_error;
}

uint8_t linesensor_get_digital(void)
{
    return g_digital;
}
