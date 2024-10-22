#include "stm32f411xx_rcc_driver.h"

uint16_t AHB_Prescalar[8] = {2u, 4u, 8u, 16u, 64u, 128u, 256u, 512u};

uint16_t APB_Prescalar[4] = {2u, 4u, 8u, 16u};

uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, systemClk, temp;
    uint8_t ahbp, apb1p, clkSrc;

    clkSrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3u);

    if (clkSrc == RCC_SYSTEM_CLOCK_STATUS_HSI_OSCILLATOR)
    {
        systemClk = RCC_HSI_CLOCK;
    }
    else if (clkSrc == RCC_SYSTEM_CLOCK_STATUS_HSE_OSCILLATOR)
    {
        systemClk = RCC_HSE_CLOCK;
    }
    else if (clkSrc == RCC_SYSTEM_CLOCK_STATUS_PLL)
    {
        systemClk = RCC_GetPLLOutputClock();
    }
    else
    {
        /* Do nothing */
    }

    /* AHB */
    temp = ((RCC->CFGR >> RCC_CFGR_HPRE) && 0xFu);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_Prescalar[temp - 8];
    }

    /* APB1 */
    temp = ((RCC->CFGR >> RCC_CFGR_PPRE1) && 0xFu);

    if (temp < 4)
    {
        apb1p = 1;
    }
    else
    {
        apb1p = APB_Prescalar[temp - 4];
    }

    pclk1 = (systemClk / ahbp) / apb1p;

    return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t pclk2, systemClk, temp;
    uint8_t ahbp, apb2p, clkSrc;

    clkSrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x03u);

    if (clkSrc == RCC_SYSTEM_CLOCK_STATUS_HSI_OSCILLATOR)
    {
        systemClk = RCC_HSI_CLOCK;
    }
    else if (clkSrc == RCC_SYSTEM_CLOCK_STATUS_HSE_OSCILLATOR)
    {
        systemClk = RCC_HSE_CLOCK;
    }
    else
    {
        /* Do nothing */
    }

    /* AHB */
    temp = ((RCC->CFGR >> RCC_CFGR_HPRE) && 0xF);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_Prescalar[temp - 8];
    }

    /* APB2 */
    temp = ((RCC->CFGR >> RCC_CFGR_PPRE2) && 0xF);

    if (temp < 4)
    {
        apb2p = 1;
    }
    else
    {
        apb2p = APB_Prescalar[temp - 4];
    }

    pclk2 = (systemClk / ahbp) / apb2p;

    return pclk2;
}

uint32_t RCC_GetPLLOutputClock(void)
{
    return 0;
}