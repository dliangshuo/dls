#include "led.h"

void LED_Config(void)
{
	//1）时钟使能--GPIOB、GPIOE时钟使能
	RCC->APB2ENR |= (1 << 3);
	RCC->APB2ENR |= (1 << 6);
	//2）模式配置
	GPIOB->CRL &= ~(0XF << 20);//先清零
	GPIOB->CRL |= (0X3 << 20);//0011 通用推挽输出
	GPIOE->CRL &= ~(0XF << 20);//先清零
	GPIOE->CRL |= (0X3 << 20);//0011 通用推挽输出
	//3）控制灯
	GPIOB->ODR |= (1 << 5);//关灯
	GPIOE->ODR |= (1 << 5);//关灯
}