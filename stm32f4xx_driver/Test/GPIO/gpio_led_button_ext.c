#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"

#define HIGH        1
#define LOW         0
#define BTN_PRESSED LOW

void GPIO_LED_Init(void);
void GPIO_BTN_Init(void);

int main(void)
{
    GPIO_LED_Init();

    GPIO_BTN_Init();

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

void GPIO_LED_Init(void)
{
    st_GPIO_Handle_t led;
    memset(&led, 0, sizeof(led));

    led.pGPIOx                             = GPIOA;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_8;
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

    btn.pGPIOx                             = GPIOB;
    btn.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    btn.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

    GPIO_Init(&btn);
}