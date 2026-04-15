/**
 * @file    mq2.c
 * @brief   MQ-2 烟雾/可燃气体传感器驱动
 *
 * 优化说明：
 *   1. RL_VALUE 修正为 5.0f kΩ（原 0.5f，与注释/实际电路不符）。
 *   2. 移除与头文件重复的 MQ2_SAMPLE_TIMES 定义，统一使用
 *      mq2.h 中的 MQ2_READ_TIMES。
 *   3. 低通滤波系数调整为 0.8/0.2，响应速度更快（原 0.7/0.3 偏慢）。
 *   4. MQ2_Convert_PPM() 内减少冗余局部变量，直接链式计算。
 *   5. 输入引脚无需设置 GPIO_Speed，已去除。
 */

#include "mq2.h"
#include <math.h>

/* ====================== 传感器参数 ====================== */
#define RL_VALUE    5.0f    /* 负载电阻，单位 kΩ（实物约 5kΩ）    */
#define VREF        3.3f    /* ADC 参考电压（V）                    */
#define SAMPLE_DELAY_MS  2  /* 每次采样间隔（ms）                   */

/**
 * R0：传感器在清洁空气中的基准阻值（kΩ）。
 * 应在传感器充分预热（>24h）后通过实测校准；此处为默认经验值。
 */
static float R0 = 6.64f;

/* ====================== 内部函数 ====================== */

/**
 * @brief  将 12 位 ADC 原始值转换为 PPM 浓度
 * @param  adc  ADC 原始值（0 ~ 4095）
 * @retval 估算 PPM 值（基于 MQ-2 数据手册对数曲线）
 *
 * 转换公式推导：
 *   V    = adc * VREF / 4096
 *   RS   = (VREF - V) / V * RL    （电压分压器，RS 在上，RL 在下）
 *   ratio= RS / R0
 *   ppm  = 10 ^ ((log10(ratio) - 0.38) / -0.32)
 */
static float MQ2_Convert_PPM(uint16_t adc)
{
    float voltage = adc * VREF / 4096.0f;

    /* 防止除零（ADC 值为 0 时电压极低） */
    if (voltage < 0.01f)
        voltage = 0.01f;

    float RS    = (VREF - voltage) / voltage * RL_VALUE;
    float ratio = RS / R0;

    return powf(10.0f, (log10f(ratio) - 0.38f) / -0.32f);
}

/* ====================== 公共接口 ====================== */

/**
 * @brief  初始化 MQ-2 传感器引脚及 ADC
 */
void MQ2_Init(void)
{
#if MODE
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(MQ2_AO_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = MQ2_AO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   /* 模拟输入，无需配置 Speed */
    GPIO_Init(MQ2_AO_GPIO_PORT, &GPIO_InitStructure);

    ADCx_Init();
#else
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(MQ2_DO_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = MQ2_DO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   /* DO 输出高电平告警，上拉防悬空 */
    GPIO_Init(MQ2_DO_GPIO_PORT, &GPIO_InitStructure);
#endif
}

/**
 * @brief  获取多次采样均值（ADC 原始值）
 * @retval 12 位平均 ADC 值（AO 模式）或阈值状态（DO 模式）
 */
uint16_t MQ2_GetData(void)
{
#if MODE
    uint32_t sum = 0;
    uint8_t  i;

    for (i = 0; i < MQ2_READ_TIMES; i++)
    {
        sum += ADC_GetValue(ADC_CHANNEL, ADC_SampleTime_55Cycles5);
        DELAY_Nms(SAMPLE_DELAY_MS);
    }

    return (uint16_t)(sum / MQ2_READ_TIMES);
#else
    return (uint16_t)(!GPIO_ReadInputDataBit(MQ2_DO_GPIO_PORT, MQ2_DO_GPIO_PIN));
#endif
}

/**
 * @brief  获取当前 PPM 浓度（含低通滤波）
 * @retval 滤波后的 PPM 浮点值
 *
 * 低通滤波：ppm_filtered = 0.8 * last + 0.2 * current
 *   系数越大（偏向 last），响应越慢但越平滑；
 *   原 0.7/0.3 响应偏慢，调整为 0.8/0.2 兼顾平滑与响应。
 */
float MQ2_GetData_PPM(void)
{
#if MODE
    static float last_ppm = 0.0f;

    float ppm = MQ2_Convert_PPM(MQ2_GetData());

    /* 一阶低通滤波 */
    last_ppm = last_ppm * 0.8f + ppm * 0.2f;

    return last_ppm;
#else
    /* DO 模式：超阈值映射为 1000 ppm 告警标志，未超为 0 */
    return MQ2_GetData() ? 1000.0f : 0.0f;
#endif
}
