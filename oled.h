#ifndef _OLED_H_
#define _OLED_H_

#include "stm32f10x.h"
#include "softiic.h"
#include "delay.h"
#include "font.h"  // 已包含字库头文件

//定义屏幕大小
#define OLED_WIGH 128
#define OLED_PAGE 8

#define OLED_CLOSE 	0X00
#define OLED_OPEN		0XFF	


typedef enum
{
	OLED_CMD=0,
	OLED_DATA
}OLED_MODE;

uint8_t OLED_ReadWriteByte(uint8_t Data,OLED_MODE Cmd);

void OLED_Config(void);
void OLED_SetPos(uint16_t x, uint16_t y);
void OLED_Clear(uint8_t data);
void OLED_ShowASCII(uint8_t page,uint8_t line,uint8_t *buf);
void OLED_ShowChinese(uint8_t page,uint8_t line,uint8_t *buf);
void OLED_ShowPhoto(uint8_t page,uint8_t line,uint8_t High,uint8_t Wide,uint8_t *buf);
void OLED_ShowString(uint8_t page,uint8_t line,uint8_t *str);

// -------------------------- 新增字库和函数声明 --------------------------
// 外部引用ASCII字库和中文字库
extern const uint8_t ASCIIFont_Lib[][16];
extern const uint8_t ChineseFont_Lib[][32];

// 字符偏移计算函数声明
extern uint8_t ASCII_GetOffset(uint8_t c);
extern uint8_t Chinese_GetOffset(uint8_t *ch);
// ------------------------------------------------------------------------

#endif
