#include "stm32f411xe_gpio_driver.h"

void GPIO_PeriClockControl(st_GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else
        {
            /* Do nothing */
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else
        {
            /* Do nothing */
        }
    }
}

void GPIO_Init(st_GPIO_Handle_t *pGPIOHandle)
{
    /* Enable Peripheral Clock */
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    uint32_t temp = 0;

    /* Configure the mode of GPIO Pin */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        /* The non interrupt mode */
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        /* The interrupt mode */
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            /* Set the FTSR bit */
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            /* Clear the RTSR bit */
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            /* Set the RTSR bit */
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            /* Clear the FTSR bit */
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            /* Set the FTSR bit */
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            /* Set the RTSR bit */
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else
        {
            /* Do nothing */
        }

        /* Configure the GPIO Port selection in the SYSCFG EXTICR */
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] &= ~(0xF << (temp2 * 4));
        SYSCFG->EXTICR[temp1] |= port_code << (temp2 * 4);

        /* Enable the EXTI interrupt delivery */
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    temp = 0;

    /* Configure the speed of output of GPIO Pin */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    /* Configure the Pull-Up Pull-Down settings of GPIO Pin */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    /* Configure the output type of GPIO Pin */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    /* Configure the Alternate functionality of GPIO Pin */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        /* Configure the alt function registers */
        uint32_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (temp2)));
    }
}

void GPIO_DeInit(st_GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
}

uint8_t GPIO_ReadFromInputPin(st_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

uint16_t GPIO_ReadFromInputPort(st_GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

void GPIO_WriteToOutputPin(st_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        /* Write 1 to the output data register */
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        /* Write 0 to the output data register */
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(st_GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(st_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber < 32)
        {
            /* Enable NVIC ISER0 Register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber < 64)
        {
            /* Enable NVIC ISER1 Register */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Enable NVIC ISER2 Register */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
        else if (IRQNumber >= 96 && IRQNumber < 128)
        {
            /* Enable NVIC ISER3 Register */
            *NVIC_ISER3 |= (1 << (IRQNumber % 96));
        }
        else if (IRQNumber >= 128 && IRQNumber < 160)
        {
            /* Enable NVIC ISER4 Register */
            *NVIC_ISER4 |= (1 << (IRQNumber % 128));
        }
        else if (IRQNumber >= 160 && IRQNumber < 192)
        {
            /* Enable NVIC ISER5 Register */
            *NVIC_ISER5 |= (1 << (IRQNumber % 160));
        }
        else if (IRQNumber >= 192 && IRQNumber < 224)
        {
            /* Enable NVIC ISER6 Register */
            *NVIC_ISER6 |= (1 << (IRQNumber % 192));
        }
        else if (IRQNumber >= 224 && IRQNumber < 256)
        {
            /* Enable NVIC ISER7 Register */
            *NVIC_ISER7 |= (1 << (IRQNumber % 224));
        }
        else
        {
            /* Do nothing */
        }
    }
    else
    {
        if (IRQNumber < 32)
        {
            /* Disable NVIC ICER0 Register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber < 64)
        {
            /* Disable NVIC ICER1 Register */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Disable NVIC ICER2 Register */
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
        else if (IRQNumber >= 96 && IRQNumber < 128)
        {
            /* Disable NVIC ICER3 Register */
            *NVIC_ICER3 |= (1 << (IRQNumber % 96));
        }
        else if (IRQNumber >= 128 && IRQNumber < 160)
        {
            /* Disable NVIC ICER4 Register */
            *NVIC_ICER4 |= (1 << (IRQNumber % 128));
        }
        else if (IRQNumber >= 160 && IRQNumber < 192)
        {
            /* Disable NVIC ICER5 Register */
            *NVIC_ICER5 |= (1 << (IRQNumber % 160));
        }
        else if (IRQNumber >= 192 && IRQNumber < 224)
        {
            /* Disable NVIC ICER6 Register */
            *NVIC_ICER6 |= (1 << (IRQNumber % 192));
        }
        else if (IRQNumber >= 224 && IRQNumber < 256)
        {
            /* Disable NVIC ICER7 Register */
            *NVIC_ICER7 |= (1 << (IRQNumber % 224));
        }
        else
        {
            /* Do nothing */
        }
    }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    /* First lets find out the IPR Register */
    uint8_t IPRx = IRQNumber / 4;
    uint8_t IPRx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    /* Clear the EXTI PR Register */
    if (EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber);
    }
}