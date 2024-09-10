#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"

int main(void)
{
    st_GPIO_Handle_t led;

    /* led configuration for pull-up pull-down */
    led.pGPIOx                             = GPIOA;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_8;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&led);

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
        delay(500000);
    }

    return 0;
}