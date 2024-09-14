#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"

void GPIO_LED_Init(void);

int main(void)
{
    GPIO_LED_Init();

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        delay(500000);
    }

    return 0;
}

void GPIO_LED_Init(void)
{
    st_GPIO_Handle_t led;

    led.pGPIOx                             = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&led);
}