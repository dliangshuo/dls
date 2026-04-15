#include "mq2.h"
#include <math.h>

// ================== ??????? ==================
#define MQ2_SAMPLE_TIMES   10      // ????????
#define MQ2_DELAY_MS       2       // ???????
#define RL_VALUE           0.5f    // ??????裨K????
#define VREF               3.3f    // ADC?ο????

// ?? ???????????
static float R0 = 6.64f;  

// ================== ????? ==================
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
// ================== ADC??? ==================
uint16_t MQ2_ADC_Read(void)
{
    return ADC_GetValue(ADC_CHANNEL, ADC_SampleTime_55Cycles5);
}
#endif

// ================== ????????? ==================
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

// ================== ?????????ADC?PPM ==================
static float MQ2_Convert_PPM(uint16_t adc)
{
    float voltage;
    float RS;
    float ratio;
    float ppm;

    // 1. ?????????????3.3V??
    voltage = adc * VREF / 4096.0f;

    // ?????
    if (voltage < 0.01f)
        voltage = 0.01f;

    // 2. ???????????? RS
    RS = (VREF - voltage) / voltage * RL_VALUE;

    // 3. ??????
    ratio = RS / R0;

    // 4. ??????????????
    ppm = powf(10, (log10f(ratio) - 0.38f) / -0.32f);

    return ppm;
}

// ================== ???PPM ==================
float MQ2_GetData_PPM(void)
{
#if MODE
    static float last_ppm = 0;   // ?????

    uint16_t adc = MQ2_GetData();   // ?? ??????
    float ppm = MQ2_Convert_PPM(adc);

    // ===== ??????????????????=====
    ppm = last_ppm * 0.7f + ppm * 0.3f;
    last_ppm = ppm;

    return ppm;
#else
    // DO模式仅输出阈值触发状态，映射为工程内部可用的“等效ppm”
    return MQ2_GetData() ? 1000.0f : 0.0f;
#endif
}