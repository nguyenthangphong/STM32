#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_delay.h"

int main(void)
{
    GPIO_Handle_t led;

    /* led configuration */
    led.pGPIOx                             = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_13;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&led);

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
        delay(500000);
    }

    return 0;
}