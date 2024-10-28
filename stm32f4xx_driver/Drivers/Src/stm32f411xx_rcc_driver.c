#include "stm32f411xx_rcc_driver.h"

uint32_t RCC_GetSystemClock(void)
{
    uint32_t System_Clock = 0u;

    uint8_t Clock_Status = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3u);

    if (Clock_Status == RCC_SWS_HSI_OSCILLATOR)
    {
        System_Clock = RCC_HSI_CLOCK;
    }
    else if (Clock_Status == RCC_SWS_HSE_OSCILLATOR)
    {
        System_Clock = RCC_HSE_CLOCK;
    }
    else if (Clock_Status == RCC_SWS_PLL)
    {
        System_Clock = RCC_GetPLLOutputClock();
    }
    else
    {
        /* Do nothing */
    }

    return System_Clock;
}


uint32_t RCC_GetAHBPrescaler(void)
{
    uint32_t AHBP;

    uint32_t AHB_Prescaler = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xFu);

    if (AHB_Prescaler < RCC_SYSTEM_CLOCK_DIV_2)
    {
        AHBP = 1u;
    }
    else
    {
        AHBP = RCC_AHB_PRESCALER(AHB_Prescaler);
    }

    return AHBP;
}

uint32_t RCC_GetAPB1Prescaler(void)
{
    uint32_t APB1P;

    uint32_t APB1_Prescaler = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7u);

    if (APB1_Prescaler < RCC_AHB_CLOCK_DIV_2)
    {
        APB1P = 1u;
    }
    else
    {
        APB1P = RCC_APB_PRESCALER(APB1_Prescaler);
    }

    return APB1P;
}

uint32_t RCC_GetAPB2Prescaler(void)
{
    uint32_t APB2P;

    uint32_t APB2_Prescaler = ((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7u);

    if (APB2_Prescaler < RCC_AHB_CLOCK_DIV_2)
    {
        APB2P = 1u;
    }
    else
    {
        APB2P = RCC_APB_PRESCALER(APB2_Prescaler);
    }

    return APB2P;
}

uint32_t RCC_GetAPBLowSpeedPrescaler(void)
{
    uint32_t APB_Low_Speed_Prescaler = (RCC_GetSystemClock() / RCC_GetAHBPrescaler()) / RCC_GetAPB1Prescaler();
    return APB_Low_Speed_Prescaler;
}

uint32_t RCC_GetAPBHighSpeedPrescaler(void)
{
    uint32_t APB_High_Speed_Prescaler = (RCC_GetSystemClock() / RCC_GetAHBPrescaler()) / RCC_GetAPB2Prescaler();
    return APB_High_Speed_Prescaler;
}

uint32_t RCC_GetPLLOutputClock(void)
{
    uint8_t PLLN = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLN) & 0xFFu);
    uint8_t PLLM = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLM) & 0x3Fu);
    uint8_t PLLP = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLP) & 0x03u);

    bool Oscillator_Type = ((RCC->PLLCFGR >> RCC_PLLCFGR_SRC) & 0x1u);

    uint32_t f_PLL_Clock_Input;

    if (Oscillator_Type)
    {
        f_PLL_Clock_Input = RCC_HSE_CLOCK;
    }
    else
    {
        f_PLL_Clock_Input = RCC_HSI_CLOCK;
    }

    uint32_t f_PLL_General_Clock_Output = (uint32_t)((f_PLL_Clock_Input * (PLLN / PLLM)) / RCC_PLLP_DIV_FACTOR(PLLP));

    return f_PLL_General_Clock_Output;
}