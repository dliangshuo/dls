#ifndef _BEEP_H_
#define _BEEP_H_

#include "stm32f10x.h"

#define BEEP_State(x) ((x) ? (GPIOF->ODR |= (1 << 8)) : (GPIOF->ODR &= ~(1 << 8)))
#define BEEP_Toggle   (GPIOF->ODR ^= (1 << 8))

void BEEP_Config(void);

#endif
