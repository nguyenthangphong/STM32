#include "stm32f411xe_rcc_driver.h"
#include "stm32f411xe_flash_driver.h"

uint32_t system_core_clock = 16000000U;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

e_StatusTypeDef_t RCC_OscillatorConfig(st_RCC_OscillatorInitTypeDef_t *pRCC_Oscillator)
{
    if (pRCC_Oscillator == NULL)
    {
        return STATUS_ERROR;
    }

    uint32_t system_clock_switch_status;
    uint32_t pll_clock_source_type;

    /* HSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
    {
        system_clock_switch_status = ((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS);
        pll_clock_source_type = ((RCC->CFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_POS);

        if ((system_clock_switch_status == RCC_SWS_HSE_OSCILLATOR) || \
           ((system_clock_switch_status == RCC_SWS_PLL) && (pll_clock_source_type == RCC_PLLSRC_HSE))
        )
        {
            if ((((RCC->CR & RCC_CR_HSERDY) >> RCC_CR_HSERDY_POS) != NOT_READY) && (pRCC_Oscillator->HSEState == RCC_HSE_OFF))
            {
                return STATUS_ERROR;
            }
        }
        else
        {
            /* Set the new HSE State */
            if (pRCC_Oscillator->HSEState == RCC_HSE_ON)
            {
                RCC->CR |= ((SET << RCC_CR_HSEON_POS) & RCC_CR_HSEON);
            }
            else if (pRCC_Oscillator->HSEState == RCC_HSE_BYPASS)
            {
                RCC->CR |= ((SET << RCC_CR_HSEON_POS) & RCC_CR_HSEON);
                RCC->CR |= ((SET << RCC_CR_HSEBYP_POS) & RCC_CR_HSEBYP);
            }
            else
            {
                RCC->CR &= ~((SET << RCC_CR_HSEON_POS) & RCC_CR_HSEON);
                RCC->CR &= ~((SET << RCC_CR_HSEBYP_POS) & RCC_CR_HSEBYP);
            }

            /* Check the HSE State */
            if (pRCC_Oscillator->HSEState != RCC_HSE_OFF)
            {
                /* Wait till HSE is ready */
                while (((RCC->CR & RCC_CR_HSERDY) >> RCC_CR_HSERDY_POS) == NOT_READY)
                {

                }
            }
            else
            {
                /* Wait till HSE is bypassed or disabled */
                while (((RCC->CR & RCC_CR_HSERDY) >> RCC_CR_HSERDY_POS) != NOT_READY)
                {

                }
            }
        }
    }

    /* HSI Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
    {
        system_clock_switch_status = ((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS);
        pll_clock_source_type = ((RCC->CFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_POS);
        
        if ((system_clock_switch_status == RCC_SWS_HSI_OSCILLATOR) || \
           ((system_clock_switch_status == RCC_SWS_PLL) && (pll_clock_source_type == RCC_PLLSRC_HSI))
        )
        {
            if ((((RCC->CR & RCC_CR_HSIRDY) >> RCC_CR_HSIRDY_POS) != NOT_READY) && (pRCC_Oscillator->HSIState == RCC_HSI_OFF))
            {
                return STATUS_ERROR;
            }
            else
            {
                /* Adjusts the Internal High Speed oscillator (HSI) calibration value */
                RCC->CR = (RCC->CR & ~(RCC_CR_HSITRIM)) | ((pRCC_Oscillator->HSICalibrationValue << RCC_CR_HSITRIM_POS) & RCC_CR_HSITRIM);
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
                while (((RCC->CR & RCC_CR_HSIRDY) >> RCC_CR_HSIRDY_POS) == NOT_READY)
                {
                    
                }

                /* Adjusts the Internal High Speed oscillator (HSI) calibration value */
                RCC->CR = (RCC->CR & ~(RCC_CR_HSITRIM)) | ((pRCC_Oscillator->HSICalibrationValue << RCC_CR_HSITRIM_POS) & RCC_CR_HSITRIM);
            }
            else
            {
                /* Disable the Internal High Speed oscillator (HSI) */
                RCC_HSI_DISABLE();

                /* Wait till HSI is bypassed or disabled */
                while (((RCC->CR & RCC_CR_HSIRDY) >> RCC_CR_HSIRDY_POS) != NOT_READY)
                {

                }
            }
        }
    }

    /* LSE Configuration */
    if (((pRCC_Oscillator->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
    {
        /* Set the new LSE State */
        if (pRCC_Oscillator->LSEState == RCC_LSE_ON)
        {
            RCC->BDCR |= ((SET << RCC_BDCR_LSEON_POS) & RCC_BDCR_LSEON);
        }
        else if (pRCC_Oscillator->LSEState == RCC_LSE_BYPASS)
        {
            RCC->BDCR |= ((SET << RCC_BDCR_LSEON_POS) & RCC_BDCR_LSEON);
            RCC->BDCR |= ((SET << RCC_BDCR_LSEBYP_POS) & RCC_BDCR_LSEBYP);
        }
        else
        {
            RCC->BDCR &= ~((SET << RCC_BDCR_LSEON_POS) & RCC_BDCR_LSEON);
            RCC->BDCR &= ~((SET << RCC_BDCR_LSEBYP_POS) & RCC_BDCR_LSEBYP);
        }

        /* Check the LSE State */
        if (pRCC_Oscillator->LSEState != RCC_LSE_OFF)
        {
            /* Wait till LSE is ready */
            while (((RCC->BDCR & RCC_BDCR_LSERDY) >> RCC_BDCR_LSERDY_POS) == NOT_READY)
            {
                
            }
        }
        else
        {
            /* Wait till LSE is bypassed or disabled */
            while (((RCC->BDCR & RCC_BDCR_LSERDY) >> RCC_BDCR_LSERDY_POS) != NOT_READY)
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
            while (((RCC->CSR & RCC_CSR_LSIRDY) & RCC_CSR_LSIRDY_POS) == NOT_READY)
            {
                
            }
        }
        else
        {
            /* Disable the Internal Low Speed oscillator (LSI) */
            RCC_LSI_DISABLE();

            /* Wait till LSI is bypassed or disabled */
            while (((RCC->CSR & RCC_CSR_LSIRDY) & RCC_CSR_LSIRDY_POS) != NOT_READY)
            {

            }
        }
    }

    /* PLL Configuration */
    if (pRCC_Oscillator->PLL.PLLState != RCC_PLL_NONE)
    {
        system_clock_switch_status = ((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS);

        /* Check if the PLL is used as system clock or not */
        if (system_clock_switch_status != RCC_SWS_PLL)
        {
            if (pRCC_Oscillator->PLL.PLLState == RCC_PLL_ON)
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                while (((RCC->CR & RCC_CR_PLLRDY) >> RCC_CR_PLLRDY_POS) != NOT_READY)
                {

                }

                /* Configure the main PLL clock source, multiplication and division factos */
                RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLSRC)) | \
                    ((pRCC_Oscillator->PLL.PLLSource << RCC_PLLCFGR_PLLSRC_POS) & RCC_PLLCFGR_PLLSRC);

                RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLM)) | \
                    ((pRCC_Oscillator->PLL.PLLM << RCC_PLLCFGR_PLLM_POS) & RCC_PLLCFGR_PLLM);

                RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLN)) | \
                    ((pRCC_Oscillator->PLL.PLLN << RCC_PLLCFGR_PLLN_POS) & RCC_PLLCFGR_PLLN);

                RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLP)) | \
                    ((pRCC_Oscillator->PLL.PLLP << RCC_PLLCFGR_PLLP_POS) & RCC_PLLCFGR_PLLP);

                /* Enable the main PLL */
                RCC_PLL_ENABLE();

                /* Wait till PLL is ready */
                while (((RCC->CR & RCC_CR_PLLRDY) >> RCC_CR_PLLRDY_POS) == NOT_READY)
                {

                }
            }
            else
            {
                /* Disable the main PLL */
                RCC_PLL_DISABLE();

                /* Wait till PLL is disabled */
                while (((RCC->CR & RCC_CR_PLLRDY) >> RCC_CR_PLLRDY_POS) != NOT_READY)
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
                if ((pRCC_Oscillator->PLL.PLLState  == RCC_PLL_ON)                                                      || \
                    (pRCC_Oscillator->PLL.PLLSource != ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_POS)) || \
                    (pRCC_Oscillator->PLL.PLLM      != ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_POS))     || \
                    (pRCC_Oscillator->PLL.PLLN      != ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_POS))     || \
                    (pRCC_Oscillator->PLL.PLLP      != ((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_POS))
                )
                {
                    return STATUS_ERROR;
                }
            }
        }
    }

    return STATUS_OK;
}

e_StatusTypeDef_t RCC_ClockConfig(st_RCC_ClockInitTypeDef_t *pRCC_Clock, uint32_t f_Latency)
{
    if (pRCC_Clock == NULL)
    {
        return STATUS_ERROR;
    }

    /* Increasing the number of wait states because of higher CPU frequency */
    if (((FLASH->ACR & FLASH_ACR_LATENCY) >> FLASH_ACR_LATENCY_POS) < f_Latency)
    {
        FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY)) | ((f_Latency << FLASH_ACR_LATENCY_POS) & FLASH_ACR_LATENCY);

        if (((FLASH->ACR & FLASH_ACR_LATENCY) >> FLASH_ACR_LATENCY_POS) != f_Latency)
        {
            return STATUS_ERROR;
        }
    }

    /* HCLK Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
    {
        if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
        {
            RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PPRE1)) | ((RCC_APB1_PRESCALER_16 << RCC_CFGR_PPRE1_POS) & RCC_CFGR_PPRE1);
        }

        if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
        {
            RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PPRE2)) | ((RCC_APB2_PRESCALER_16 << RCC_CFGR_PPRE2_POS) & RCC_CFGR_PPRE2);
        }

        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE)) | ((pRCC_Clock->AHB_ClockDivider << RCC_CFGR_HPRE_POS) & RCC_CFGR_HPRE);
    }

    /* SYSCLK Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
    {
        /* HSE is selected as System Clock Source */
        if (pRCC_Clock->SystemClockSource == RCC_SYSCLKSOURCE_HSE)
        {
            /* Check the HSE ready flag */
            if (((RCC->CR & RCC_CR_HSERDY) >> RCC_CR_HSERDY_POS) == NOT_READY)
            {
                return STATUS_ERROR;
            }
        }
        /* PLL is selected as System Clock Source */
        else if (pRCC_Clock->SystemClockSource == RCC_SYSCLKSOURCE_PLLCLK)
        {
            /* Check the PLL ready flag */
            if (((RCC->CR & RCC_CR_PLLRDY) >> RCC_CR_PLLRDY_POS) == NOT_READY)
            {
                return STATUS_ERROR;
            }
        }
        /* HSI is selected as System Clock Source */
        else
        {
            /* Check the HSI ready flag */
            if (((RCC->CR & RCC_CR_HSIRDY) >> RCC_CR_HSIRDY_POS) == NOT_READY)
            {
                return STATUS_ERROR;
            }
        }

        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | ((pRCC_Clock->SystemClockSource << RCC_CFGR_SW_POS) & RCC_CFGR_SW);

        while (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) != pRCC_Clock->SystemClockSource)
        {

        }
    }

    /* Decreasing the number of wait states because of lower CPU frequency */
    if (((FLASH->ACR & FLASH_ACR_LATENCY) >> FLASH_ACR_LATENCY_POS) > f_Latency)
    {
        FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY)) | ((f_Latency << FLASH_ACR_LATENCY_POS) & FLASH_ACR_LATENCY);

        if (((FLASH->ACR & FLASH_ACR_LATENCY) >> FLASH_ACR_LATENCY_POS) != f_Latency)
        {
            return STATUS_ERROR;
        }
    }

    /* PCLK1 Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
    {
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PPRE1)) | ((pRCC_Clock->APB1_ClockDivider << RCC_CFGR_PPRE1_POS) & RCC_CFGR_PPRE1);
    }

    /* PCLK2 Configuration */
    if (((pRCC_Clock->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
    {
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PPRE2)) | ((pRCC_Clock->APB2_ClockDivider << RCC_CFGR_PPRE2_POS) & RCC_CFGR_PPRE2);
    }

    /* Update the SystemCoreClock global variable */
    system_core_clock = RCC_GetSysClockFreq() >> AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_POS)];

    return STATUS_OK;
}

void RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv)
{
    st_GPIO_Handle_t gpio_MCox = {0};

    /* MCO1 Configuration */
    if (RCC_MCOx == RCC_MCO1)
    {
        /* MCO1 Clock Enable */
        GPIOA_PCLK_EN();

        /* Configure the MCO1 Pin */
        gpio_MCox.pGPIOx                             = GPIOA;
        gpio_MCox.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_8;
        gpio_MCox.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        gpio_MCox.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
        gpio_MCox.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
        gpio_MCox.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_AF0_MCO;
        gpio_MCox.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
        GPIO_Init(&gpio_MCox);

        /* Set MCO1 Clock Source */
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO1)) | ((RCC_MCOSource << RCC_CFGR_MCO1_POS) & RCC_CFGR_MCO1);

        /* Set MCO1 Prescaler */
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO1PRE)) | ((RCC_MCODiv << RCC_CFGR_MCO1PRE_POS) & RCC_CFGR_MCO1PRE);
    }
    /* MCO2 Configuration */
    else
    {
        /* MCO2 Clock Enable */
        GPIOC_PCLK_EN();

        /* Configure the MCO2 Pin */
        gpio_MCox.pGPIOx                             = GPIOC;
        gpio_MCox.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_9;
        gpio_MCox.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        gpio_MCox.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
        gpio_MCox.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
        gpio_MCox.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_AF0_MCO;
        gpio_MCox.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
        GPIO_Init(&gpio_MCox);

        /* Set MCO2 Clock Source */
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO2)) | ((RCC_MCOSource << RCC_CFGR_MCO2_POS) & RCC_CFGR_MCO2);

        /* Set MCO2 Prescaler */
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO2PRE)) | ((RCC_MCODiv << RCC_CFGR_MCO2PRE_POS) & RCC_CFGR_MCO2PRE);
    }
}

uint32_t RCC_GetSysClockFreq(void)
{
    uint32_t system_clock_freq = 0U;

    /* HSE used as system clock source */
    if (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) == RCC_SWS_HSE_OSCILLATOR)
    {
        system_clock_freq = RCC_HSE_CLOCK;
    }
    /* PLL used as system clock source */
    else if (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) == RCC_SWS_PLL)
    {
        uint32_t pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_POS);
        uint32_t plln = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_POS);
        uint32_t pllp = RCC_PLLP_DIV_FACTOR(((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_POS));
        uint32_t f_pll_clock_input = 0U;
        uint32_t f_vco_clock = 0U;
        uint32_t f_pll_general_clock_output = 0U;

        /* HSI used as PLL clock source */
        if (((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_POS) == RCC_PLLSRC_HSI)
        {
            f_pll_clock_input = RCC_HSI_CLOCK;
        }
        /* HSE used as PLL clock source */
        else
        {
            f_pll_clock_input = RCC_HSE_CLOCK;
        }

        f_vco_clock = (f_pll_clock_input * plln) / pllm;

        f_pll_general_clock_output = f_vco_clock / pllp;

        system_clock_freq = f_pll_general_clock_output;
    }
    /* HSI used as system clock source */
    else
    {
        system_clock_freq = RCC_HSI_CLOCK;
    }

    return system_clock_freq;
}

uint32_t RCC_GetHCLKFreq(void)
{
    return system_core_clock;
}

uint32_t RCC_GetPCLK1Freq(void)
{
    return (RCC_GetHCLKFreq() >> APBPrescTable[((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_POS)]);
}

uint32_t RCC_GetPCLK2Freq(void)
{
    return (RCC_GetHCLKFreq() >> APBPrescTable[((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_POS)]);
}