#include "stm32f411xe_rcc_driver.h"

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
    uint32_t APB_Low_Speed_Prescaler = (uint32_t)((RCC_GetSystemClock() / RCC_GetAHBPrescaler()) / RCC_GetAPB1Prescaler());
    return APB_Low_Speed_Prescaler;
}

uint32_t RCC_GetAPBHighSpeedPrescaler(void)
{
    uint32_t APB_High_Speed_Prescaler = (uint32_t)((RCC_GetSystemClock() / RCC_GetAHBPrescaler()) / RCC_GetAPB2Prescaler());
    return APB_High_Speed_Prescaler;
}

uint32_t RCC_GetPLLOutputClock(void)
{
    uint8_t PLLN = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLN) & 0xFFu);
    uint8_t PLLM = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLM) & 0x3Fu);
    uint8_t PLLP = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLP) & 0x03u);

    bool Oscillator_Type = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC) & 0x1u);

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

void RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv)
{
    st_GPIO_Handle_t GPIO_MCOx;
    uint32_t temp = 0u;

    if (RCC_MCOx == RCC_MCO1)
    {
        /* MCO1 Clock Enable */
        GPIOA_PCLK_EN();

        /* Configure the MCO1 Pin */
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_8;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_AF0_MCO;
        GPIO_Init(&GPIO_MCOx);

        /* Set MCO1 Clock Source */
        temp |= (RCC_MCOSource << RCC_CFGR_MCO1);

        /* Set MCO1 Prescaler */
        temp |= (RCC_MCODiv << RCC_CFGR_MCO1PRE);

        /* Configuration CFGR Register */
        RCC->CFGR = temp;
    }
    else
    {
        /* MCO2 Clock Enable */
        GPIOC_PCLK_EN();

        /* Configure the MCO2 Pin */
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_9;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_AF0_MCO;
        GPIO_Init(&GPIO_MCOx);

        /* Set MCO2 Clock Source */
        temp |= (RCC_MCOSource << RCC_CFGR_MCO2);

        /* Set MCO2 Prescaler */
        temp |= (RCC_MCODiv << RCC_CFGR_MCO2PRE);

        /* Configuration CFGR Register */
        RCC->CFGR = temp;
    }
}

void RCC_OscillatorConfig(st_RCC_OscillatorInitTypeDef_t *pRCC_Oscillator)
{
    uint32_t Clock_Status = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3u);
    uint32_t Oscillator_Type = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC) & 0x1u);

    /* HSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
    {
        if ((Clock_Status == RCC_SWS_HSE_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSE)))
        {
            if ((RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET) && (pRCC_Oscillator->HSEState == RCC_HSE_OFF))
            {
                return;
            }
            else
            {

            }
        }
    }

    /* HSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
    {
        if ((Clock_Status == RCC_SWS_HSI_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSI)))
        {
            if ((RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != RESET) && (pRCC_Oscillator->HSIState == RCC_HSI_OFF))
            {
                return;
            }
        }
    }

    /* LSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
    {
        if ((Clock_Status == RCC_SWS_HSI_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSI)))
        {
            if ((RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != RESET) && (pRCC_Oscillator->HSIState == RCC_HSI_OFF))
            {
                return;
            }
        }
    }

    /* LSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
    {
        if ((Clock_Status == RCC_SWS_HSI_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSI)))
        {
            if ((RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != RESET) && (pRCC_Oscillator->HSIState == RCC_HSI_OFF))
            {
                return;
            }
        }
    }

    /* PLL Configuration */
}

/* 
 * RCC_Flag Flags
 * Elements values convention: 0XXYYYYY
 * YYYYY  : Flag position in the register
 *   0XX  : Register index
 *    01  : CR register
 *    10  : BDCR register
 *    11  : CSR register
 */

uint8_t RCC_GetFlagStatus(uint8_t FlagName)
{
    uint32_t register_value = 0u;
    uint8_t temp;

    if ((FlagName >> 5u) == 1u)
    {
        register_value = RCC->CR;
    }
    else if ((FlagName >> 5u) == 2u)
    {
        register_value = RCC->BDCR;
    }
    else if ((FlagName >> 5u) == 3u)
    {
        register_value = RCC->CSR;
    }
    else
    {
        /* Do nothing */
    }

    temp = (uint8_t)(((register_value & (1u << (FlagName & 0x1Fu))) != 0u) ? SET : RESET);

    return temp;
}

void RCC_HSEConfig(uint32_t HSE_State)
{
    if (HSE_State == RCC_HSE_ON)
    {
        RCC->CR |= (1 << RCC_CR_HSEON);
    }
    else if (HSE_State == RCC_HSE_BYPASS)
    {
        RCC->CR |= (1 << RCC_CR_HSEON);
        RCC->CR |= (1 << RCC_CR_HSEBYP);
    }
    else
    {
        RCC->CR &= ~(1 << RCC_CR_HSEON);
        RCC->CR &= ~(1 << RCC_CR_HSEBYP);
    }
}