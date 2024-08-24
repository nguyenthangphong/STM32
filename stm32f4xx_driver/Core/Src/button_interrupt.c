#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"

int main(void)
{
    return 0;
}

void EXTI0_IRQHandler(void)
{
    // Handle the interrupt
    GPIO_IRQHandling(GPIO_PIN_NO_0);
}