#include "stm32f411xe_rcc_driver.h"

uint32_t SystemCoreClock = 16000000U;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

e_StatusTypeDef_t RCC_OscillatorConfig(st_RCC_OscillatorInitTypeDef_t *pRCC_Oscillator)
{
    if (pRCC_Oscillator == NULL)
    {
        return STATUS_ERROR;
    }

    uint32_t Clock_Status = READ_REGISTER(RCC->CFGR, RCC_CFGR_SWS);
    uint32_t Oscillator_Type = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);

    /* HSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
    {
        volatile uint32_t HSE_Status;

        if ((Clock_Status == RCC_SWS_HSE_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSE)))
        {
            HSE_Status = READ_REGISTER(RCC->CR, RCC_CR_HSERDY);

            if ((HSE_Status != RESET) && (pRCC_Oscillator->HSEState == RCC_HSE_OFF))
            {
                return STATUS_ERROR;
            }
        }
        else
        {
            /* Set the new HSE State */
            if (pRCC_Oscillator->HSEState == RCC_HSE_ON)
            {
                SET_REGISTER(RCC->CR, RCC_CR_HSEON, SET);
            }
            else if (pRCC_Oscillator->HSEState == RCC_HSE_BYPASS)
            {
                SET_REGISTER(RCC->CR, RCC_CR_HSEON, SET);
                SET_REGISTER(RCC->CR, RCC_CR_HSEBYP, SET);
            }
            else
            {
                SET_REGISTER(RCC->CR, RCC_CR_HSEON, RESET);
                SET_REGISTER(RCC->CR, RCC_CR_HSEBYP, RESET);
            }

            /* Check the HSE State */
            if (pRCC_Oscillator->HSEState != RCC_HSE_OFF)
            {
                /* Wait till HSE is ready */
                HSE_Status = READ_REGISTER(RCC->CR, RCC_CR_HSERDY);

                while (HSE_Status == RESET)
                {

                }
            }
            else
            {
                /* Wait till HSE is bypassed or disabled */
                HSE_Status = READ_REGISTER(RCC->CR, RCC_CR_HSERDY);

                while (HSE_Status != RESET)
                {

                }
            }
        }
    }

    /* HSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
    {
        volatile uint32_t HSI_Status;

        if ((Clock_Status == RCC_SWS_HSI_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSI)))
        {
            HSI_Status = READ_REGISTER(RCC->CR, RCC_CR_HSIRDY);

            if ((HSI_Status != RESET) && (pRCC_Oscillator->HSIState == RCC_HSI_OFF))
            {
                return STATUS_ERROR;
            }
            else
            {
                /* Adjusts the Internal High Speed oscillator (HSI) calibration value */
                SET_REGISTER(RCC->CR, RCC_CR_HSITRIM, pRCC_Oscillator->HSICalibrationValue);
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
                HSI_Status = READ_REGISTER(RCC->CR, RCC_CR_HSIRDY);

                while (HSI_Status == RESET)
                {
                    
                }

                /* Adjusts the Internal High Speed oscillator (HSI) calibration value */
                SET_REGISTER(RCC->CR, RCC_CR_HSITRIM, pRCC_Oscillator->HSICalibrationValue);
            }
            else
            {
                /* Disable the Internal High Speed oscillator (HSI) */
                RCC_HSI_DISABLE();

                /* Wait till HSI is bypassed or disabled */
                HSI_Status = READ_REGISTER(RCC->CR, RCC_CR_HSIRDY);

                while (HSI_Status != RESET)
                {

                }
            }
        }
    }

    /* LSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
    {
        volatile uint32_t LSE_Status;

        /* Set the new LSE State */
        if (pRCC_Oscillator->LSEState == RCC_LSE_ON)
        {
            SET_REGISTER(RCC->BDCR, RCC_BDCR_LSEON, SET);
        }
        else if (pRCC_Oscillator->LSEState == RCC_LSE_BYPASS)
        {
            SET_REGISTER(RCC->BDCR, RCC_BDCR_LSEON, SET);
            SET_REGISTER(RCC->BDCR, RCC_BDCR_LSEBYP, SET);
        }
        else
        {
            SET_REGISTER(RCC->BDCR, RCC_BDCR_LSEON, RESET);
            SET_REGISTER(RCC->BDCR, RCC_BDCR_LSEBYP, RESET);
        }

        /* Check the LSE State */
        if (pRCC_Oscillator->LSEState != RCC_LSE_OFF)
        {
            /* Wait till LSE is ready */
            LSE_Status = READ_REGISTER(RCC->BDCR, RCC_BDCR_LSERDY);

            while (LSE_Status == RESET)
            {
                
            }
        }
        else
        {
            /* Wait till LSE is bypassed or disabled */
            LSE_Status = READ_REGISTER(RCC->BDCR, RCC_BDCR_LSERDY);

            while (LSE_Status != RESET)
            {
                
            }
        }
    }

    /* LSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
    {
        volatile uint32_t LSI_Status;

        /* Check the LSI State */
        if (pRCC_Oscillator->LSIState == RCC_LSI_ON)
        {
            /* Enable the Internal Low Speed oscillator (LSI) */
            RCC_LSI_ENABLE();

            /* Wait till LSI is ready */
            LSI_Status = READ_REGISTER(RCC->CSR, RCC_CSR_LSIRDY);

            while (LSI_Status == RESET)
            {
                
            }
        }
        else
        {
            /* Disable the Internal Low Speed oscillator (LSI) */
            RCC_LSI_DISABLE();

            /* Wait till LSI is bypassed or disabled */
            LSI_Status = READ_REGISTER(RCC->CSR, RCC_CSR_LSIRDY);

            while (LSI_Status != RESET)
            {

            }
        }
    }

    /* PLL Configuration */
    if (pRCC_Oscillator->PLL.PLLState != RCC_PLL_NONE)
    {
        volatile uint32_t PLL_Status;
        uint8_t System_Clock_Source = READ_REGISTER(RCC->CFGR, RCC_CFGR_SWS);

        /* Check if the PLL is used as system clock or not */
        if (System_Clock_Source != RCC_SWS_PLL)
        {
            if (pRCC_Oscillator->PLL.PLLState == RCC_PLL_ON)
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                PLL_Status = READ_REGISTER(RCC->CR, RCC_CR_PLLRDY);

                while (PLL_Status != RESET)
                {

                }

                /* Clear the PLLCFGR register */
                CLEAR_REGISTER(RCC->PLLCFGR);

                /* Configure the main PLL clock source, multiplication and division factos */
                SET_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, pRCC_Oscillator->PLL.PLLSource);
                SET_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLM  , pRCC_Oscillator->PLL.PLLM);
                SET_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLN  , pRCC_Oscillator->PLL.PLLN);
                SET_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLP  , pRCC_Oscillator->PLL.PLLP);

                /* Enable the main PLL */
                RCC_PLL_ENABLE();

                /* Wait till PLL is ready */
                PLL_Status = READ_REGISTER(RCC->CR, RCC_CR_PLLRDY);

                while (PLL_Status == RESET)
                {

                }
            }
            else
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                PLL_Status = READ_REGISTER(RCC->CR, RCC_CR_PLLRDY);

                while (PLL_Status != RESET)
                {

                }
            }
        }
        else
        {
            /* Check if there is a request to disable the PLL used as System clock source */
            if (pRCC_Oscillator->PLL.PLLState == RCC_PLL_OFF)
            {
                return STATUS_ERROR;
            }
            else
            {
                uint32_t PLLSrc_Cur = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);
                uint32_t PLLM_Cur   = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLM);
                uint32_t PLLN_Cur   = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLN);
                uint32_t PLLP_Cur   = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLP);

                if ((pRCC_Oscillator->PLL.PLLState == RCC_PLL_ON) || \
                    (pRCC_Oscillator->PLL.PLLSource != PLLSrc_Cur) || (pRCC_Oscillator->PLL.PLLM != PLLM_Cur) || \
                    (pRCC_Oscillator->PLL.PLLN != PLLN_Cur) || (pRCC_Oscillator->PLL.PLLP != PLLP_Cur)
                )
                {
                    return STATUS_ERROR;
                }
            }
        }
    }

    return STATUS_OK;
}

e_StatusTypeDef_t RCC_ClockConfig(st_RCC_ClockInitTypeDef_t *pRCC_Clock)
{
    if (pRCC_Clock == NULL)
    {
        return STATUS_ERROR;
    }

    /* HCLK Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
    {
        if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
        {
            SET_BIT(RCC->CFGR, RCC_APB1_PRESCALER_16);
        }

        if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
        {
            SET_BIT(RCC->CFGR, RCC_APB2_PRESCALER_16);
        }

        SET_BIT(RCC->CFGR, pRCC_Clock->AHB_ClockDivider);
    }

    /* SYSCLK Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
    {
        /* HSE is selected as System Clock Source */
        if (pRCC_Clock->SystemClockSource == RCC_SYSCLKSOURCE_HSE)
        {
            /* Check the HSE ready flag */
            if (READ_REGISTER(RCC->CR, RCC_CR_HSERDY) == RESET)
            {
                return STATUS_ERROR;
            }
        }
        /* PLL is selected as System Clock Source */
        else if (pRCC_Clock->SystemClockSource == RCC_SYSCLKSOURCE_PLLCLK)
        {
            /* Check the PLL ready flag */
            if (READ_REGISTER(RCC->CR, RCC_CR_PLLRDY) == RESET)
            {
                return STATUS_ERROR;
            }
        }
        /* HSI is selected as System Clock Source */
        else
        {
            /* Check the HSI ready flag */
            if (READ_REGISTER(RCC->CR, RCC_CR_HSIRDY) == RESET)
            {
                return STATUS_ERROR;
            }
        }

        SET_REGISTER(RCC->CFGR, RCC_CFGR_SW, pRCC_Clock->SystemClockSource);

        while (READ_REGISTER(RCC->CFGR, RCC_CFGR_SWS) != pRCC_Clock->SystemClockSource)
        {

        }
    }

    /* PCLK1 Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
    {
        SET_BIT(RCC->CFGR, pRCC_Clock->APB1_ClockDivider);
    }

    /* PCLK2 Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
    {
        SET_BIT(RCC->CFGR, pRCC_Clock->APB2_ClockDivider);
    }

    /* Update the SystemCoreClock global variable */
    SystemCoreClock = RCC_GetSysClockFreq() >> AHBPrescTable[READ_REGISTER(RCC->CFGR, RCC_CFGR_HPRE)];

    return STATUS_OK;
}

void RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv)
{
    st_GPIO_Handle_t GPIO_MCOx;

    if (RCC_MCOx == RCC_MCO1)
    {
        /* MCO1 Clock Enable */
        GPIOA_PCLK_EN();

        /* Configure the MCO1 Pin */
        GPIO_MCOx.pGPIOx                             = GPIOA;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_8;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_AF0_MCO;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
        GPIO_Init(&GPIO_MCOx);

        /* Set MCO1 Clock Source */
        SET_REGISTER(RCC->CFGR, RCC_CFGR_MCO1, RCC_MCOSource);

        /* Set MCO1 Prescaler */
        SET_REGISTER(RCC->CFGR, RCC_CFGR_MCO1PRE, RCC_MCODiv);
    }
    else
    {
        /* MCO2 Clock Enable */
        GPIOC_PCLK_EN();

        /* Configure the MCO2 Pin */
        GPIO_MCOx.pGPIOx                             = GPIOC;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_9;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_AF0_MCO;
        GPIO_MCOx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
        GPIO_Init(&GPIO_MCOx);

        /* Set MCO2 Clock Source */
        SET_REGISTER(RCC->CFGR, RCC_CFGR_MCO2, RCC_MCOSource);

        /* Set MCO2 Prescaler */
        SET_REGISTER(RCC->CFGR, RCC_CFGR_MCO2PRE, RCC_MCODiv);
    }
}

uint32_t RCC_GetSysClockFreq(void)
{
    uint32_t SWS_Status;
    uint32_t f_PLL_clock_input, f_VCO_clock, f_PLL_general_clock_output;
    uint32_t PLLN, PLLM, PLLP;

    SWS_Status = READ_REGISTER(RCC->CFGR, RCC_CFGR_SWS);

    if (SWS_Status == RCC_SWS_HSI_OSCILLATOR)
    {
        f_PLL_clock_input = RCC_HSI_CLOCK;
    }
    else
    {
        f_PLL_clock_input = RCC_HSE_CLOCK;
    }

    PLLM = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLM);
    PLLN = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLN);
    PLLP = READ_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLP);

    f_VCO_clock = (f_PLL_clock_input / PLLM) * PLLN;

    PLLP = RCC_PLLP_DIV_FACTOR(PLLP);
    f_PLL_general_clock_output = f_VCO_clock / PLLP;

    return f_PLL_general_clock_output;
}

uint32_t RCC_GetHCLKFreq(void)
{
    return SystemCoreClock;
}

uint32_t RCC_GetPCLK1Freq(void)
{
    uint32_t f_APB1 = 0x00000000U;
    f_APB1 = (RCC_GetHCLKFreq() >> APBPrescTable[READ_REGISTER(RCC->CFGR, RCC_CFGR_PPRE1)]);
    return f_APB1;
}

uint32_t RCC_GetPCLK2Freq(void)
{
    uint32_t f_APB2 = 0x00000000U;
    f_APB2 = (RCC_GetHCLKFreq() >> APBPrescTable[READ_REGISTER(RCC->CFGR, RCC_CFGR_PPRE2)]);
    return f_APB2;
}