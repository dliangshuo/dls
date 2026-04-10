#ifndef _ADC_H_
#define _ADC_H_

#include "stm32f10x.h"
#include "delay.h"

#define SR602_GPIO_PORT GPIOA
#define SR602_GPIO_PIN GPIO_Pin_0
#define SR602_GPIO_CLK RCC_APB2Periph_GPIOA

void SR602_Init(void);
uint8_t SR602_Detect(void);

#endif // _SERIAL_H_
