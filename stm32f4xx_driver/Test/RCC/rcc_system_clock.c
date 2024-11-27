#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_rcc_driver.h"
#include "stm32f411xe_flash_driver.h"
#include "delay.h"

extern uint32_t system_core_clock;

void RCC_SystemClockConfigHSI(void);
void RCC_SystemClockConfigHSE(void);

int main(void)
{
    RCC_SystemClockConfigHSE();
    RCC_MCOConfig(RCC_MCO1, RCC_MCO1_PLL_CLOCK_SOURCE, RCC_MCO1_PRESCALER_DIV_1);

    return 0;
}

/*
 *  16_000_000 Hz => PLL => 16_000_000 * 160 / 40 / 8  = 8_000_000 Hz
 *  APB1 = 8_000_000 / 8 = 1_000_000 Hz
 *  APB2 = 8_000_000 / 4 = 2_000_000 Hz
 *  MC01 = 8_000_000 / 2 = 4_000_000 Hz => T = 1 / 4_000_000 = 250(ns)
 *  f_sampling >= 2 * system_core_clock = 2 * 8_000_000 = 16 MS/s
 *  f_sampling >= 5 * system_core_clock (pulse waveform) = 40 MS/s
 */
void RCC_SystemClockConfigHSI(void)
{
    e_StatusTypeDef_t ret = STATUS_OK;

    st_RCC_OscillatorInitTypeDef_t rcc_oscillator_config = {0};

    rcc_oscillator_config.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    rcc_oscillator_config.HSIState            = RCC_HSI_ON;
    rcc_oscillator_config.HSICalibrationValue = RCC_HSITRIM_6;
    rcc_oscillator_config.PLL.PLLState        = RCC_PLL_ON;
    rcc_oscillator_config.PLL.PLLSource       = RCC_PLLSRC_HSI;
    rcc_oscillator_config.PLL.PLLM            = RCC_PLLM_DIV_40;
    rcc_oscillator_config.PLL.PLLN            = RCC_PLLN_MUL_160;
    rcc_oscillator_config.PLL.PLLP            = RCC_PLLP_DIV_8;

    ret = RCC_OscillatorConfig(&rcc_oscillator_config);

    st_RCC_ClockInitTypeDef_t rcc_clock_config = {0};

    rcc_clock_config.ClockType                = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    rcc_clock_config.SystemClockSource        = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clock_config.APB1_ClockDivider        = RCC_APB1_PRESCALER_8;
    rcc_clock_config.APB2_ClockDivider        = RCC_APB2_PRESCALER_4;

    ret = RCC_ClockConfig(&rcc_clock_config, FLASH_LATENCY_2);

    UNUSED(ret);
}

/*
 *  8_000_000 Hz => PLL => 8_000_000 * 360 / 40 / 6  = 12_000_000 Hz
 *  APB1 = 12_000_000 / 2 = 6_000_000 Hz
 *  APB2 = 12_000_000 / 4 = 3_000_000 Hz
 *  MC01 = 12_000_000 / 2 = 6_000_000 Hz => T = 1 / 6_000_000 = 167(ns)
 *  f_sampling >= 2 * system_core_clock = 2 * 12_000_000 = 24 MS/s
 *  f_sampling >= 5 * system_core_clock (pulse waveform) = 60 MS/s
 */
void RCC_SystemClockConfigHSE(void)
{
    e_StatusTypeDef_t ret = STATUS_OK;

    st_RCC_OscillatorInitTypeDef_t rcc_oscillator_config = {0};

    rcc_oscillator_config.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    rcc_oscillator_config.HSEState            = RCC_HSE_ON;
    rcc_oscillator_config.PLL.PLLState        = RCC_PLL_ON;
    rcc_oscillator_config.PLL.PLLSource       = RCC_PLLSRC_HSE;
    rcc_oscillator_config.PLL.PLLM            = RCC_PLLM_DIV_40;
    rcc_oscillator_config.PLL.PLLN            = RCC_PLLN_MUL_360;
    rcc_oscillator_config.PLL.PLLP            = RCC_PLLP_DIV_6;

    ret = RCC_OscillatorConfig(&rcc_oscillator_config);

    st_RCC_ClockInitTypeDef_t rcc_clock_config = {0};

    rcc_clock_config.ClockType                = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    rcc_clock_config.SystemClockSource        = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clock_config.APB1_ClockDivider        = RCC_APB1_PRESCALER_2;
    rcc_clock_config.APB2_ClockDivider        = RCC_APB2_PRESCALER_4;

    ret = RCC_ClockConfig(&rcc_clock_config, FLASH_LATENCY_1);

    UNUSED(ret);
}