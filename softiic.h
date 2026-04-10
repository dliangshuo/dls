#ifndef _SOFTIIC_H_
#define _SOFTIIC_H_

#include "stm32f10x.h"
#include "delay.h"

/*
接线说明
	管脚	模块
	VCC		VCC
	GND		GND
	SCL		SCL
	SDA		SDA 			
*/

//IICSCLK
#define IICSCL_ClockCmd RCC_APB2PeriphClockCmd
#define IICSCL_Periph RCC_APB2Periph_GPIOD
#define IICSCL_PORT GPIOD
#define IICSCL_PIN GPIO_Pin_3
//IICSDA
#define IICSDA_ClockCmd RCC_APB2PeriphClockCmd
#define IICSDA_Periph RCC_APB2Periph_GPIOG
#define IICSDA_PORT GPIOG
#define IICSDA_PIN GPIO_Pin_13


#define SOFT_SCL(x)	(x)?(IICSCL_PORT->ODR |= IICSCL_PIN):(IICSCL_PORT->ODR &= ~IICSCL_PIN)
#define SOFT_SDA(x)	(x)?(IICSDA_PORT->ODR |= IICSDA_PIN):(IICSDA_PORT->ODR &= ~IICSDA_PIN)	

void SoftIIC_Config(void);
void IIC_WriteCommand(uint8_t command);
void IIC_WriteData(uint8_t data);
uint8_t SOFTSP0_ReadAndWrite(uint8_t write_data); // IIC读写（解决L6218E: SOFTSP0_ReadAndWrite）
void OLED_Write(uint8_t cmd, uint8_t data);       // OLED专用写（解决L6218E: OLED_Write）

#endif
