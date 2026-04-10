#ifndef _FONT_H_
#define _FONT_H_

#include "stm32f10x.h"

// ASCII字库声明（8x16点阵）
extern const uint8_t ASCIIFont_Lib[][16];

// 中文字库声明（16x16点阵）
extern const uint8_t ChineseFont_Lib[][32];

// ASCII字符偏移计算函数
uint8_t ASCII_GetOffset(uint8_t c);

// 中文字符偏移计算函数（参数为字符指针）
uint8_t Chinese_GetOffset(uint8_t *ch);

#endif
