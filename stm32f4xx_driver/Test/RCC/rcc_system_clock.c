#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_rcc_driver.h"
#include "stm32f411xe_flash_driver.h"
#include "delay.h"

extern uint32_t SystemCoreClock;

void RCC_SystemClockConfigHSI(void);
void RCC_SystemClockConfigHSE(void);

int main(void)
{
    RCC_SystemClockConfigHSE();

    volatile uint32_t f_APB1;
    volatile uint32_t f_APB2;

    f_APB1 = RCC_GetPCLK1Freq();
    f_APB2 = RCC_GetPCLK2Freq();

    UNUSED(f_APB1);
    UNUSED(f_APB2);

    return 0;
}

/*
 *  16_000_000 Hz => PLL => 16_000_000 * 72 / 16 = 72_000_000 Hz
 *  APB1 = 72_000_000 / 8 = 9_000_000 Hz
 *  APB2 = 72_000_000 / 4 = 18_000_000 Hz
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
    rcc_oscillator_config.PLL.PLLM            = RCC_PLLM_DIV_4;
    rcc_oscillator_config.PLL.PLLN            = RCC_PLLN_MUL_72;
    rcc_oscillator_config.PLL.PLLP            = RCC_PLLP_DIV_4;

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
 *  8_000_000 Hz => PLL => 8_000_000 * 72 / 16 = 36_000_000 Hz
 *  APB1 = 36_000_000 / 8 = 4_500_000 Hz
 *  APB2 = 36_000_000 / 4 = 9_000_000 Hz
 */
void RCC_SystemClockConfigHSE(void)
{
    e_StatusTypeDef_t ret = STATUS_OK;

    st_RCC_OscillatorInitTypeDef_t rcc_oscillator_config = {0};

    rcc_oscillator_config.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    rcc_oscillator_config.HSEState            = RCC_HSE_ON;
    rcc_oscillator_config.PLL.PLLState        = RCC_PLL_ON;
    rcc_oscillator_config.PLL.PLLSource       = RCC_PLLSRC_HSE;
    rcc_oscillator_config.PLL.PLLM            = RCC_PLLM_DIV_4;
    rcc_oscillator_config.PLL.PLLN            = RCC_PLLN_MUL_72;
    rcc_oscillator_config.PLL.PLLP            = RCC_PLLP_DIV_4;

    ret = RCC_OscillatorConfig(&rcc_oscillator_config);

    st_RCC_ClockInitTypeDef_t rcc_clock_config = {0};

    rcc_clock_config.ClockType                = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    rcc_clock_config.SystemClockSource        = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clock_config.APB1_ClockDivider        = RCC_APB1_PRESCALER_8;
    rcc_clock_config.APB2_ClockDivider        = RCC_APB2_PRESCALER_4;

    ret = RCC_ClockConfig(&rcc_clock_config, FLASH_LATENCY_1);

    UNUSED(ret);
}