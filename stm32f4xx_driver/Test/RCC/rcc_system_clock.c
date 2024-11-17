#include <string.h>
#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_rcc_driver.h"
#include "delay.h"

void GPIO_LED_Init(void);
void RCC_SystemClockConfig(void);

int main(void)
{
    RCC_SystemClockConfig();
    RCC_MCOConfig(RCC_MCO1, RCC_MCO1_HSI_CLOCK_SOURCE, RCC_MCO1_PRESCALER_DIV_2);
    GPIO_LED_Init();

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_14);
        delay(1000);
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
        delay(1000);
    }

    return 0;
}

void GPIO_LED_Init(void)
{
    st_GPIO_Handle_t led = {0};
    led.pGPIOx                             = GPIOB;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_14;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&led);
}

void RCC_SystemClockConfig(void)
{
    e_StatusTypeDef_t ret;

    st_RCC_OscillatorInitTypeDef_t rcc_system_clock_config = {0};
    rcc_system_clock_config.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    rcc_system_clock_config.HSIState            = RCC_HSI_ON;
    rcc_system_clock_config.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_system_clock_config.PLL.PLLState        = RCC_PLL_NONE;

    ret = RCC_OscillatorConfig(&rcc_system_clock_config);
    UNUSED(ret);
}