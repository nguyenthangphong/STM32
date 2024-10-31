#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_rcc_driver.h"
#include "delay.h"

int main(void)
{
    volatile uint32_t ahb_prescaler = RCC_GetAHBPrescaler();
    UNUSED(ahb_prescaler);

    volatile uint32_t abp1_prescaler = RCC_GetAPB1Prescaler();
    UNUSED(abp1_prescaler);

    volatile uint32_t abp2_prescaler = RCC_GetAPB2Prescaler();
    UNUSED(abp2_prescaler);

    volatile uint32_t system_clock = RCC_GetSystemClock();
    UNUSED(system_clock);

    volatile uint32_t apb1_clock = RCC_GetAPBLowSpeedPrescaler();
    UNUSED(apb1_clock);

    volatile uint32_t apb2_clock = RCC_GetAPBHighSpeedPrescaler();
    UNUSED(apb2_clock);

    volatile uint32_t pll_output_clock = RCC_GetPLLOutputClock();
    UNUSED(pll_output_clock);

    RCC_MCOConfig(RCC_MCO1, RCC_MCO1_HSI_CLOCK, RCC_SYSTEM_CLOCK_DIV_2);

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
        delay(1000);
    }

    return 0;
}