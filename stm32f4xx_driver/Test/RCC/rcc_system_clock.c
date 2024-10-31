#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_rcc_driver.h"
#include "delay.h"

void GPIO_MCO1_Init(void);
void GPIO_MCO2_Init(void);

int main(void)
{
    GPIO_MCO1_Init();
    GPIO_MCO2_Init();

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
        delay(1000);
        GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_9);
        delay(1000);
    }

    return 0;
}

/* MCO1 Pin => PA8 */
void GPIO_MCO1_Init(void)
{
    st_GPIO_Handle_t mco1;

    mco1.pGPIOx                             = GPIOA;
    mco1.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_8;
    mco1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    mco1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    mco1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    mco1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&mco1);
}

/* MCO2 Pin => PC9 */
void GPIO_MCO2_Init(void)
{
    st_GPIO_Handle_t mco2;

    mco2.pGPIOx                             = GPIOC;
    mco2.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_9;
    mco2.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    mco2.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    mco2.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    mco2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&mco2);
}