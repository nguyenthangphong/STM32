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
    uint32_t PLL_Config;

    /* HSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
    {
        if ((Clock_Status == RCC_SWS_HSE_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSE)))
        {
            if ((RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET) && (pRCC_Oscillator->HSEState == RCC_HSE_OFF))
            {
                return;
            }
        }
        else
        {
            /* Set the new HSE State */
            RCC_HSEConfig(pRCC_Oscillator->HSEState);

            /* Check the HSE State */
            if (pRCC_Oscillator->HSEState != RCC_HSE_OFF)
            {
                /* Wait till HSE is ready */
                while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
                {

                }
            }
            else
            {
                /* Wait till HSE is bypassed or disabled */
                while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
                {

                }
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
        else
        {
            /* Check the HSI State */
            if (pRCC_Oscillator->HSIState == RCC_HSI_ON)
            {
                /* Enable the Internal High Speed oscillator (HSI) */
                RCC_HSI_ENABLE();

                /* Wait till HSI is ready */
                while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
                {
                    
                }

                /* Adjusts the Internal High Speed oscillator (HSI) calibration value */
                RCC->CR |= (pRCC_Oscillator->HSICalibrationValue << RCC_CR_HSITRIM);
            }
            else
            {
                /* Disable the Internal High Speed oscillator (HSI) */
                RCC_HSI_DISABLE();

                /* Wait till HSI is bypassed or disabled */
                while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != RESET)
                {

                }
            }
        }
    }

    /* LSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
    {
        /* Set the new LSE State */
        RCC_LSEConfig(pRCC_Oscillator->LSEState);

        /* Check the LSE State */
        if (pRCC_Oscillator->LSEState != RCC_LSE_OFF)
        {
            /* Wait till LSE is ready */
            while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
            {
                
            }
        }
        else
        {
            /* Wait till LSE is bypassed or disabled */
            while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != RESET)
            {
                
            }
        }
    }

    /* LSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
    {
        /* Check the LSI State */
        if (pRCC_Oscillator->LSIState == RCC_LSI_ON)
        {
            /* Enable the Internal Low Speed oscillator (LSI) */
            RCC_LSI_ENABLE();

            /* Wait till LSI is ready */
            while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
            {
                
            }
        }
        else
        {
            /* Disable the Internal Low Speed oscillator (LSI) */
            RCC_LSI_DISABLE();

            /* Wait till LSI is bypassed or disabled */
            while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != RESET)
            {

            }
        }
    }

    /* PLL Configuration */
    if (pRCC_Oscillator->PLL.PLLState != RCC_PLL_NONE)
    {
        uint8_t System_Clock_Source = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3u);

        /* Check if the PLL is used as system clock or not */
        if (System_Clock_Source != RCC_SWS_PLL)
        {
            if (pRCC_Oscillator->PLL.PLLState == RCC_PLL_ON)
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET)
                {

                }

                /* Configure the main PLL clock source, multiplication and division factos */
                RCC->PLLCFGR |= (pRCC_Oscillator->PLL.PLLSource << RCC_PLLCFGR_PLLSRC);
                RCC->PLLCFGR |= (pRCC_Oscillator->PLL.PLLM << RCC_PLLCFGR_PLLM);
                RCC->PLLCFGR |= (pRCC_Oscillator->PLL.PLLN << RCC_PLLCFGR_PLLN);
                RCC->PLLCFGR |= (((pRCC_Oscillator->PLL.PLLP >> 1u) - 1u) << RCC_PLLCFGR_PLLP);
                RCC->PLLCFGR |= (pRCC_Oscillator->PLL.PLLQ << RCC_PLLCFGR_PLLQ);

                /* Enable the main PLL */
                RCC_PLL_ENABLE();

                /* Wait till PLL is ready */
                while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
                {

                }
            }
            else
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET)
                {

                }
            }
        }
        else
        {
            /* Check if there is a request to disable the PLL used as System clock source */
            if (pRCC_Oscillator->PLL.PLLState == RCC_PLL_OFF)
            {
                return;
            }
            else
            {
                uint32_t pllsrc = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC) & 0x1u);
                uint32_t pllm = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLM) & 0x3Fu);
                uint32_t plln = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLN) & 0x1FFu);
                uint32_t pllp = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLP) & 0x2u);
                uint32_t pllq = ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLQ) & 0xFu);

                if ((pRCC_Oscillator->PLL.PLLState == RCC_PLL_OFF) || 
                    (pllsrc != pRCC_Oscillator->PLL.PLLSource)     ||
                    (pllm   |= pRCC_Oscillator->PLL.PLLM)          ||
                    (plln   |= pRCC_Oscillator->PLL.PLLN)          ||
                    (pllp   |= pRCC_Oscillator->PLL.PLLP)          ||
                    (pllq   |= pRCC_Oscillator->PLL.PLLQ)
                )
                {
                    return;
                }
            }
        }
    }
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

void RCC_HSIConfig(uint32_t HSI_State)
{
    if (HSI_State == RCC_HSI_ON)
    {
        RCC->CR |= (1 << RCC_CR_HSION);
    }
    else
    {
        RCC->CR &= ~(1 << RCC_CR_HSION);
    }
}

void RCC_LSEConfig(uint32_t LSE_State)
{
    if (LSE_State == RCC_LSE_ON)
    {
        RCC->BDCR |= (1 << RCC_BDCR_LSEON);
    }
    else if (LSE_State == RCC_LSE_BYPASS)
    {
        RCC->BDCR |= (1 << RCC_BDCR_LSEON);
        RCC->BDCR |= (1 << RCC_BDCR_LSEBYP);
    }
    else
    {
        RCC->BDCR &= ~(1 << RCC_BDCR_LSEON);
        RCC->BDCR &= ~(1 << RCC_BDCR_LSEBYP);
    }
}

void RCC_LSIConfig(uint32_t LSI_State)
{
    if (LSI_State == RCC_LSI_ON)
    {
        RCC->CSR |= (1 << RCC_CSR_LSION);
    }
    else
    {
        RCC->CSR &= ~(1 << RCC_CSR_LSION);
    }
}