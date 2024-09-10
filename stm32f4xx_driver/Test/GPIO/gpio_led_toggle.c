#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"

int main(void)
{
    st_GPIO_Handle_t led_1, led_2;

    /* led_1 configuration for pull-up pull-down */
    led_1.pGPIOx                             = GPIOD;
    led_1.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    led_1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led_1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led_1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led_1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&led_1);

    /* led_2 configuration for open drain */
    led_2.pGPIOx                             = GPIOD;
    led_2.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_13;
    led_2.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led_2.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led_2.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_OD;
    led_2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&led_2);

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        delay(500000);
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
        delay(500000);
    }

    return 0;
}