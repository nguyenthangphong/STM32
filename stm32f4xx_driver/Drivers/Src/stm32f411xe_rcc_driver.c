#include "stm32f411xe_rcc_driver.h"

e_StatusTypeDef_t RCC_OscillatorConfig(st_RCC_OscillatorInitTypeDef_t *pRCC_Oscillator)
{
    if (pRCC_Oscillator == NULL)
    {
        return STATUS_ERROR;
    }

    uint32_t Clock_Status = GET_REGISTER(RCC->CFGR, RCC_CFGR_SWS);
    uint32_t Oscillator_Type = GET_REGISTER(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);
    uint32_t PLL_Config;

    /* HSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
    {
        uint32_t HSE_Status;

        if ((Clock_Status == RCC_SWS_HSE_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSE)))
        {
            HSE_Status = GET_REGISTER(RCC->CR, RCC_CR_HSERDY);

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
                HSE_Status = GET_REGISTER(RCC->CR, RCC_CR_HSERDY);

                while (HSE_Status == RESET)
                {

                }
            }
            else
            {
                /* Wait till HSE is bypassed or disabled */
                HSE_Status = GET_REGISTER(RCC->CR, RCC_CR_HSERDY);

                while (HSE_Status != RESET)
                {

                }
            }
        }
    }

    /* HSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
    {
        uint32_t HSI_Status;

        if ((Clock_Status == RCC_SWS_HSI_OSCILLATOR) || ((Clock_Status == RCC_SWS_PLL) && (Oscillator_Type == RCC_PLLSRC_HSI)))
        {
            HSI_Status = ((RCC->CR >> RCC_CR_HSERDY) & 0x1u);

            if ((HSI_Status != RESET) && (pRCC_Oscillator->HSIState == RCC_HSI_OFF))
            {
                return STATUS_ERROR;
            }
            else
            {
                /* Adjusts the Internal High Speed oscillator (HSI) calibration value */
                RCC->CR |= (pRCC_Oscillator->HSICalibrationValue << RCC_CR_HSITRIM);
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
                HSI_Status = ((RCC->CR >> RCC_CR_HSERDY) & 0x1u);

                while (HSI_Status == RESET)
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
                HSI_Status = ((RCC->CR >> RCC_CR_HSERDY) & 0x1u);

                while (HSI_Status != RESET)
                {

                }
            }
        }
    }

    /* LSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
    {
        uint32_t LSE_Status;

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
            LSE_Status = GET_REGISTER(RCC->BDCR, RCC_BDCR_LSERDY);

            while (LSE_Status == RESET)
            {
                
            }
        }
        else
        {
            /* Wait till LSE is bypassed or disabled */
            LSE_Status = GET_REGISTER(RCC->BDCR, RCC_BDCR_LSERDY);

            while (LSE_Status != RESET)
            {
                
            }
        }
    }

    /* LSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
    {
        uint32_t LSI_Status;

        /* Check the LSI State */
        if (pRCC_Oscillator->LSIState == RCC_LSI_ON)
        {
            /* Enable the Internal Low Speed oscillator (LSI) */
            RCC_LSI_ENABLE();

            /* Wait till LSI is ready */
            LSI_Status = GET_REGISTER(RCC->CSR, RCC_CSR_LSIRDY);

            while (LSI_Status == RESET)
            {
                
            }
        }
        else
        {
            /* Disable the Internal Low Speed oscillator (LSI) */
            RCC_LSI_DISABLE();

            /* Wait till LSI is bypassed or disabled */
            LSI_Status = GET_REGISTER(RCC->CSR, RCC_CSR_LSIRDY);

            while (LSI_Status != RESET)
            {

            }
        }
    }

    /* PLL Configuration */
    if (pRCC_Oscillator->PLL.PLLState != RCC_PLL_NONE)
    {
        uint32_t PLL_Status;
        uint8_t System_Clock_Source = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3u);

        /* Check if the PLL is used as system clock or not */
        if (System_Clock_Source != RCC_SWS_PLL)
        {
            if (pRCC_Oscillator->PLL.PLLState == RCC_PLL_ON)
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                PLL_Status = ((RCC->CR >> RCC_CR_PLLRDY) & 0x1u);

                while (PLL_Status != RESET)
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
                PLL_Status = ((RCC->CR >> RCC_CR_PLLRDY) & 0x1u);

                while (PLL_Status == RESET)
                {

                }
            }
            else
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                PLL_Status = ((RCC->CR >> RCC_CR_PLLRDY) & 0x1u);

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
                if ((pRCC_Oscillator->PLL.PLLState  == RCC_PLL_OFF)                                     || \
                    (pRCC_Oscillator->PLL.PLLSource != ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC) & 0x1u))   || \
                    (pRCC_Oscillator->PLL.PLLM      != ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLM) & 0x3Fu))    || \
                    (pRCC_Oscillator->PLL.PLLN      != ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLN) & 0x1FFu))   || \
                    (pRCC_Oscillator->PLL.PLLP      != ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLP) & 0x2u))     || \
                    (pRCC_Oscillator->PLL.PLLQ      != ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLQ) & 0xFu))
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
            RCC->CFGR |= (RCC_CFGR_PPRE1_DIV_16 << RCC_CFGR_PPRE1);
        }
    }
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
        GPIO_MCOx.pGPIOx                             = GPIOA;
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
        GPIO_MCOx.pGPIOx                             = GPIOC;
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
