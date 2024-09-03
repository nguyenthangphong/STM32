#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>

#define HIGH        1
#define LOW         0
#define BTN_PRESSED LOW

int main(void)
{
    st_GPIO_Handle_t led, btn;

    memset(&led, 0, sizeof(led));
    memset(&btn, 0, sizeof(btn));

    /* led configuration */
    led.pGPIOx                             = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_LOW;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&led);

    /* btn configuration */
    btn.pGPIOx                             = GPIOA;
    btn.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IT_FT;
    btn.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&btn);

    GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);

    /* IRQ configuration */
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, IRQ_NO_PRIORITY_15);
    GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);

    while (1);

    return 0;
}

void EXTI9_5_IRQHandler(void)
{
    delay(500000);
    // Clear the pending event from EXTI line
    GPIO_IRQHandling(GPIO_PIN_NO_0);
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}