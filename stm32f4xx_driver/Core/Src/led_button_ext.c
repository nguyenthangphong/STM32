#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"

#define HIGH        1
#define LOW         0
#define BTN_PRESSED LOW

int main(void)
{
    GPIO_Handle_t led, btn;

    /* led configuration */
    led.pGPIOx                             = GPIOA;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_8;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&led);

    /* btn configuration */
    btn.pGPIOx                             = GPIOB;
    btn.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    btn.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_Init(&btn);

    while (1)
    {
        if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
        {
            delay(500000);
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
        }
    }

    return 0;
}