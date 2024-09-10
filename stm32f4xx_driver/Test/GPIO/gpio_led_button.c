#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"

#define HIGH 1
#define BTN_PRESSED HIGH

int main(void)
{
    st_GPIO_Handle_t led, btn;

    /* led configuration */
    led.pGPIOx                             = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&led);

    /* btn configuration */
    btn.pGPIOx                             = GPIOA;
    btn.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&btn);

    while (1)
    {
        if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
        {
            delay(500000);
            GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        }
    }

    return 0;
}