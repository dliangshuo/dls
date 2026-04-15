#include "adc.h"

/**
 * @brief  初始化 SR602 PIR 传感器 GPIO（PC4，浮空输入）
 */
void SR602_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(SR602_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = SR602_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   /* 浮空输入，输出电平由传感器驱动 */
    GPIO_Init(SR602_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief  读取 SR602 检测状态
 * @retval 1 = 检测到人体（高电平）
 *         0 = 未检测到人体（低电平）
 */
uint8_t SR602_Detect(void)
{
    return (GPIO_ReadInputDataBit(SR602_GPIO_PORT, SR602_GPIO_PIN) == Bit_SET) ? 1U : 0U;
}
