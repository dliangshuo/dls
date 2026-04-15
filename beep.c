#include "beep.h"

/**
 * @brief  初始化蜂鸣器引脚 PF0 为推挽输出，初始为关闭状态
 */
void BEEP_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能 GPIOF 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    /* 默认关闭蜂鸣器 */
    BEEP_State(0);
}
