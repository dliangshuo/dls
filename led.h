#ifndef _LED_H_
#define _LED_H_

#include "stm32f10x.h"

/*
	宏定义
	三目运算符
	枚举
*/

typedef enum{
	CLOSE=0,
	OPEN
}_IOCtrl;

#define D0_Ctrl(x) (x)?(GPIOB->ODR &= ~(1 << 5)):(GPIOB->ODR |= (1 << 5))
#define D1_Ctrl(x) (x)?(GPIOE->ODR &= ~(1 << 5)):(GPIOE->ODR |= (1 << 5))

#define D0_TOGGLE (GPIOB->ODR ^= (1 << 5))
#define D1_TOGGLE (GPIOE->ODR ^= (1 << 5))

void LED_Config(void);

#endif
