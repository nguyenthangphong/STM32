#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_delay.h"

void config_GPIOx(GPIO_Handle_t *pHandle, 
    uint8_t pin_number, uint8_t pin_mode, uint8_t pin_speed, uint8_t pin_out_type, uint8_t pin_pupd_control);

int main(void)
{
    /* LED TOGGLE */
    GPIO_Handle_t led_1, led_2, led_3, led_4;

    led_1.pGPIOx = GPIOD;
    led_2.pGPIOx = GPIOD;
    led_3.pGPIOx = GPIOD;
    led_4.pGPIOx = GPIOD;

    config_GPIOx(&led_1, GPIO_PIN_NO_12, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    config_GPIOx(&led_2, GPIO_PIN_NO_13, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    config_GPIOx(&led_3, GPIO_PIN_NO_14, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    config_GPIOx(&led_4, GPIO_PIN_NO_15, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    
    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&led_1);
    GPIO_Init(&led_2);
    GPIO_Init(&led_3);
    GPIO_Init(&led_4);

    while (1)
    {
        // Green
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        delay(500000);
        // Orange
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
        delay(500000);
        // Red
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
        delay(500000);
        // Blue
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
        delay(500000);
    }

    return 0;
}

void config_GPIOx(GPIO_Handle_t *pHandle, 
    uint8_t pin_number, uint8_t pin_mode, uint8_t pin_speed, uint8_t pin_out_type, uint8_t pin_pupd_control
)
{
    pHandle->GPIO_PinConfig.GPIO_PinNumber      = pin_number;
    pHandle->GPIO_PinConfig.GPIO_PinMode        = pin_mode;
    pHandle->GPIO_PinConfig.GPIO_PinSpeed       = pin_speed;
    pHandle->GPIO_PinConfig.GPIO_PinOPType      = pin_out_type;
    pHandle->GPIO_PinConfig.GPIO_PinPuPdControl = pin_pupd_control;
}