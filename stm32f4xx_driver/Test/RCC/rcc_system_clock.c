#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_rcc_driver.h"
#include "delay.h"

extern uint32_t SystemCoreClock;

void RCC_SystemClockConfig(void);

int main(void)
{
    RCC_SystemClockConfig();

    volatile uint32_t f_APB1;
    volatile uint32_t f_APB2;

    f_APB1 = RCC_GetPCLK1Freq();
    f_APB2 = RCC_GetPCLK2Freq();

    UNUSED(f_APB1);
    UNUSED(f_APB2);

    return 0;
}

void RCC_SystemClockConfig(void)
{
    e_StatusTypeDef_t ret = STATUS_OK;

    st_RCC_OscillatorInitTypeDef_t rcc_oscillator_config = {0};

    rcc_oscillator_config.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    rcc_oscillator_config.HSIState            = RCC_HSI_ON;
    rcc_oscillator_config.HSICalibrationValue = RCC_HSITRIM_16;
    rcc_oscillator_config.PLL.PLLState        = RCC_PLL_ON;
    rcc_oscillator_config.PLL.PLLSource       = RCC_PLLSRC_HSI;
    rcc_oscillator_config.PLL.PLLM            = RCC_PLLM_DIV_8;
    rcc_oscillator_config.PLL.PLLN            = RCC_PLLN_MUL_50;
    rcc_oscillator_config.PLL.PLLP            = RCC_PLLP_DIV_8;

    ret = RCC_OscillatorConfig(&rcc_oscillator_config);

    st_RCC_ClockInitTypeDef_t rcc_clock_config = {0};

    rcc_clock_config.ClockType                = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clock_config.SystemClockSource        = RCC_SYSCLKSOURCE_HSI;
    rcc_clock_config.AHB_ClockDivider         = RCC_AHB_PRESCALER_16;
    rcc_clock_config.APB1_ClockDivider        = RCC_APB1_PRESCALER_2;
    rcc_clock_config.APB2_ClockDivider        = RCC_APB2_PRESCALER_4;

    ret = RCC_ClockConfig(&rcc_clock_config);

    UNUSED(ret);
}