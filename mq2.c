#include "mq2.h"
#include <math.h>

// ================== 可调参数 ==================
#define MQ2_SAMPLE_TIMES   10      // 采样次数
#define MQ2_DELAY_MS       2       // 采样间隔
#define RL_VALUE           0.5f    // 负载电阻（KΩ）
#define VREF               3.3f    // ADC参考电压

// ?? 建议后期做标定
static float R0 = 6.64f;  

// ================== 初始化 ==================
void MQ2_Init(void)
{
#if MODE
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(MQ2_AO_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = MQ2_AO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(MQ2_AO_GPIO_PORT, &GPIO_InitStructure);

    ADCx_Init();
#else
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(MQ2_DO_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = MQ2_DO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(MQ2_DO_GPIO_PORT, &GPIO_InitStructure);
#endif
}

#if MODE
// ================== ADC读取 ==================
uint16_t MQ2_ADC_Read(void)
{
    return ADC_GetValue(ADC_CHANNEL, ADC_SampleTime_55Cycles5);
}
#endif

// ================== 获取原始数据 ==================
uint16_t MQ2_GetData(void)
{
#if MODE
    uint32_t sum = 0;

    for (uint8_t i = 0; i < MQ2_SAMPLE_TIMES; i++)
    {
        sum += MQ2_ADC_Read();
        DELAY_Nms(MQ2_DELAY_MS);
    }

    return (uint16_t)(sum / MQ2_SAMPLE_TIMES);
#else
    return !GPIO_ReadInputDataBit(MQ2_DO_GPIO_PORT, MQ2_DO_GPIO_PIN);
#endif
}

// ================== 内部函数：ADC转PPM ==================
static float MQ2_Convert_PPM(uint16_t adc)
{
    float voltage;
    float RS;
    float ratio;
    float ppm;

    // 1. 电压转换（修正为3.3V）
    voltage = adc * VREF / 4096.0f;

    // 防止异常
    if (voltage < 0.01f)
        voltage = 0.01f;

    // 2. 计算传感器电阻 RS
    RS = (VREF - voltage) / voltage * RL_VALUE;

    // 3. 计算比值
    ratio = RS / R0;

    // 4. 曲线拟合（烟雾）
    ppm = powf(10, (log10f(ratio) - 0.38f) / -0.32f);

    return ppm;
}

// ================== 获取PPM ==================
float MQ2_GetData_PPM(void)
{
#if MODE
    static float last_ppm = 0;   // 滤波用

    uint16_t adc = MQ2_GetData();   // ?? 只采一次
    float ppm = MQ2_Convert_PPM(adc);

    // ===== 一阶滤波（提升稳定性）=====
    ppm = last_ppm * 0.7f + ppm * 0.3f;
    last_ppm = ppm;

    return ppm;
#endif
}