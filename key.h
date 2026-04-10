#ifndef _KEY_H_
#define _KEY_H_

#include "stm32f10x.h"
#include "delay.h"

#define WKUP_State !!(GPIOA->IDR & (1 << 0))
#define KEY0_State !!(GPIOE->IDR & (1 << 4))
#define KEY1_State !!(GPIOE->IDR & (1 << 3))

void KEY_Config(void);
uint8_t KEY_GetVal(void);

#endif
