#include "adc.h"

void SR602_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIO时钟
    RCC_APB2PeriphClockCmd(SR602_GPIO_CLK, ENABLE);
    
    // 配置GPIO为输入模式
    GPIO_InitStructure.GPIO_Pin = SR602_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SR602_GPIO_PORT, &GPIO_InitStructure);
}

// 检测人体移动
uint8_t SR602_Detect(void)
{
    // 当检测到人体时，模块输出高电平
    if(GPIO_ReadInputDataBit(SR602_GPIO_PORT, SR602_GPIO_PIN) == Bit_SET)
    {
        return 1; // 检测到人体
    }
    else
    {
        return 0; // 未检测到人体
    }
}
