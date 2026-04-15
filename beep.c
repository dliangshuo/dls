#include "beep.h"

void BEEP_Config(void)
{
	//GPIOF端口时钟使能
	RCC->APB2ENR |= (1 << 7);
	//GPIOF Pin0配置为通用推挽输出
	GPIOF->CRL &= ~(0XF << 0);
	GPIOF->CRL |= (0X3 << 0);
}
