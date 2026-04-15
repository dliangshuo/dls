#include "adc.h"

void SR602_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ???GPIO???
    RCC_APB2PeriphClockCmd(SR602_GPIO_CLK, ENABLE);
    
    // PIR DQ口作为数字输入（PC4）
    GPIO_InitStructure.GPIO_Pin = SR602_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SR602_GPIO_PORT, &GPIO_InitStructure);
}

// ??????????
uint8_t SR602_Detect(void)
{
    // ??????????????????????
    if(GPIO_ReadInputDataBit(SR602_GPIO_PORT, SR602_GPIO_PIN) == Bit_SET)
    {
        return 1; // ???????
    }
    else
    {
        return 0; // δ???????
    }
}
