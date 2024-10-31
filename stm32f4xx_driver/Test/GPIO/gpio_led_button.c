#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "delay.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void GPIO_LED_Init(void);
void GPIO_BTN_Init(void);

int main(void)
{
    GPIO_BTN_Init();

    GPIO_LED_Init();

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

void GPIO_BTN_Init(void)
{
    st_GPIO_Handle_t btn;

    btn.pGPIOx                             = GPIOA;
    btn.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&btn);
}