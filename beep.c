#include "beep.h"

void BEEP_Config(void)
{
    // GPIOF port clock enable
    RCC->APB2ENR |= (1 << 7);
    // GPIOF Pin8 as general push-pull output
    GPIOF->CRH &= ~(0xF << 0);
    GPIOF->CRH |= (0x3 << 0);
}