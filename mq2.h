#ifndef __MQ2_H
#define	__MQ2_H
#include "stm32f10x.h"
#include "adcx.h"
#include "delay.h"
#include "math.h"


#define MQ2_READ_TIMES	10  //MQ-2??????ADC??????????

//?????	
//???AO:	1
//????DO:	0
#define	MODE 	1

/***************??????????????****************/
// MQ-2 GPIO????
#if MODE
#define		MQ2_AO_GPIO_CLK								RCC_APB2Periph_GPIOC
#define 	MQ2_AO_GPIO_PORT							GPIOC
// Use PC0 for MQ2 AO to avoid conflicts with key/UART/WiFi pins.
#define 	MQ2_AO_GPIO_PIN								GPIO_Pin_0
#define   ADC_CHANNEL               		ADC_Channel_10	// ADC ???????

#else
#define		MQ2_DO_GPIO_CLK								RCC_APB2Periph_GPIOA
#define 	MQ2_DO_GPIO_PORT							GPIOA
#define 	MQ2_DO_GPIO_PIN								GPIO_Pin_1			

#endif
/*********************END**********************/


void MQ2_Init(void);
uint16_t MQ2_GetData(void);
float MQ2_GetData_PPM(void);

#endif /* __ADC_H */