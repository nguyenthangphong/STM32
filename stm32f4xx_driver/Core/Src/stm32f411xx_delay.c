#include "stm32f411xx_delay.h"

void delay(uint32_t seconds)
{
    for (uint32_t i = 0; i < seconds / 2; i++);
}