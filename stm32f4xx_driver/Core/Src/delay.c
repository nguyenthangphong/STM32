#include "delay.h"

void delay(uint32_t seconds)
{
    for (uint32_t i = 0; i < seconds / 2; i++);
}