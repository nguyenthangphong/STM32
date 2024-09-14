#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>

void GPIO_LED_Init(void);
void GPIO_BTN_Init(void);

int main(void)
{
    GPIO_BTN_Init();

    GPIO_LED_Init();

    GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, IRQ_NO_PRIORITY_15);

    while (1);

    return 0;
}

void GPIO_LED_Init(void)
{
    st_GPIO_Handle_t led;
    memset(&led, 0, sizeof(led));

    led.pGPIOx                             = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&led);
}

void GPIO_BTN_Init(void)
{
    st_GPIO_Handle_t btn;
    memset(&btn, 0, sizeof(btn));

    btn.pGPIOx                             = GPIOA;
    btn.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IT_FT;
    btn.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&btn);
}

void EXTI0_IRQHandler(void)
{
    delay(500000);
    GPIO_IRQHandling(GPIO_PIN_NO_0);
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
    delay(500000);
}