#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_rcc_driver.h"
#include "delay.h"

void RCC_SystemClockConfig(void);

int main(void)
{
    RCC_SystemClockConfig();
    volatile uint32_t f_PLL_general_clock_output;
    f_PLL_general_clock_output = RCC_GetSysClockFreq();
    UNUSED(f_PLL_general_clock_output);

    return 0;
}

void RCC_SystemClockConfig(void)
{
    e_StatusTypeDef_t ret = STATUS_OK;

    st_RCC_OscillatorInitTypeDef_t rcc_system_clock_config = {0};
    rcc_system_clock_config.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    rcc_system_clock_config.HSIState            = RCC_HSI_ON;
    rcc_system_clock_config.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_system_clock_config.PLL.PLLState        = RCC_PLL_ON;
    rcc_system_clock_config.PLL.PLLSource       = RCC_PLLSRC_HSI;
    rcc_system_clock_config.PLL.PLLM            = RCC_PLLM_DIV_8;
    rcc_system_clock_config.PLL.PLLN            = RCC_PLLN_MUL_50;
    rcc_system_clock_config.PLL.PLLP            = RCC_PLLP_DIV_8;

    ret = RCC_OscillatorConfig(&rcc_system_clock_config);
    UNUSED(ret);
}