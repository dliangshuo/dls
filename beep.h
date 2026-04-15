#ifndef _BEEP_H_
#define _BEEP_H_

#include "stm32f10x.h"

#define BEEP_State(x) (x)?(GPIOF->ODR |= (1 << 0)):(GPIOF->ODR &= ~(1 << 0))

#define BEEP_Toggle (GPIOF->ODR ^= (1 << 0))

void BEEP_Config(void);

#endif
