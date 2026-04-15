#ifndef _ADC_H_
#define _ADC_H_

#include "stm32f10x.h"
#include "delay.h"

#define SR602_GPIO_PORT GPIOC
#define SR602_GPIO_PIN GPIO_Pin_4
#define SR602_GPIO_CLK RCC_APB2Periph_GPIOC

void SR602_Init(void);
uint8_t SR602_Detect(void);

#endif // _SERIAL_H_
