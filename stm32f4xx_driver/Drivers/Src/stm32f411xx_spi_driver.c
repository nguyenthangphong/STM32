#include "stm32f411xx_spi_driver.h"
#include <stdio.h>

void SPI_PeriClockControl(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
        else
        {
            /* Do nothing */
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
        else
        {
            /* Do nothing */
        }
    }
}

void SPI_Init(st_SPI_Handle_t *pSPIHandle)
{
    uint32_t temp = 0;

    /* Enable SPI for the Peripheral Clock */
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    /* Configure the Device Mode */
    temp |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

    /* Configure the Bug Config */
    if (pSPIHandle->SPI_Config.SPI_BugConfig == SPI_BUS_CONFIG_FULLDUPLEX)
    {
        /* Full Duplex -> Clear BIDIMODE bit */
        temp &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPI_Config.SPI_BugConfig == SPI_BUS_CONFIG_HALFDUPLEX)
    {
        /* Half Duplex -> Set BIDIMODE bit */
        temp |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPI_Config.SPI_BugConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        /* Simplex -> Clear BIDIMODE bit and Set RXONLY bit */
        temp &= ~(1 << SPI_CR1_BIDIMODE);
        temp |= (1 << SPI_CR1_RXONLY);
    }
    else
    {
        /* Do nothing */
    }

    /* Configure the SCLK Speed */
    temp |= (pSPIHandle->SPI_Config.SPI_SCLKSpeed << SPI_CR1_BR);

    /* Configure the DFF */
    temp |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

    /* Configure the CPOL */
    temp |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

    /* Configure the CPHA */
    temp |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

    /* Configure the SSM */
    temp |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

    /* Configuration CR1 Register */
    pSPIHandle->pSPIx->CR1 = temp;
}

void SPI_DeInit(st_SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
    else if (pSPIx == SPI4)
    {
        SPI4_REG_RESET();
    }
    else if (pSPIx == SPI5)
    {
        SPI5_REG_RESET();
    }
    else
    {
        /* Do nothing */
    }
}

void SPI_SendData(st_SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{
    while (length > 0)
    {
        /* Wait until the Tx buffer is empty */
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        /* Check DFF bit in CR1 register */
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            /* 16 bits DFF, write data to DR register */
            pSPIx->DR = *(uint16_t *)pTxBuffer;
            length--;
            length--;
            (uint16_t *)pTxBuffer++;
        }
        else
        {
            /* 8 bits DFF, write data to DR register */
            pSPIx->DR = *pTxBuffer;
            length--;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveData(st_SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{
    while (length > 0)
    {
        /* Wait until the Rx buffer is full */
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        /* Check DFF bit int CR1 register */
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            /* 16 bits DFF, read data from DR register */
            *((uint16_t *)pRxBuffer) = pSPIx->DR;
            length--;
            length--;
            (uint16_t *)pRxBuffer++;
        }
        else
        {
            /* 8 bits DFF, read data from DR register */
            *(pRxBuffer) = pSPIx->DR;
            length--;
            pRxBuffer++;
        }
    }
}

uint8_t SPI_SendDataIT(st_SPI_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t length)
{
    uint8_t state = pHandle->TxState;

    if (state != SPI_BUSY_IN_TX)
    {
        /* Save the Tx buffer address and length information in some global variables */
        pHandle->pTxBuffer = pTxBuffer;
        pHandle->TxLen = length;

        /* Mark the SPI state as busy in transmission */
        pHandle->TxState = SPI_BUSY_IN_TX;

        /* Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR register */
        pHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }

    return state;
}

uint8_t SPI_ReceiveDataIT(st_SPI_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t length)
{
    uint8_t state = pHandle->RxState;

    if (state != SPI_BUSY_IN_RX)
    {
        /* Save the Rx buffer address and length information in some global variables */
        pHandle->pRxBuffer = pRxBuffer;
        pHandle->RxLen = length;

        /* Mark the SPI state as busy in transmission */
        pHandle->RxState = SPI_BUSY_IN_RX;

        /* Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR register */
        pHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return state;
}

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    /* First lets find out the IPR Register */
    uint8_t IPRx = IRQNumber / 4;
    uint8_t IPRx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(st_SPI_Handle_t *pHandle)
{
    /* Check the TXE flag */
	uint8_t temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	uint8_t temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		/* Handle TXE error */
		SPI_TXE_Interrupt_Handle(pHandle);
	}

    /* Check the RXNE flag */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		/* Handle RXNE error */
		SPI_RXNE_Interrupt_Handle(pHandle);
	}

	/* Check the OVR flag */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		/* Handle OVR error */
		SPI_OVR_Interrupt_Handle(pHandle);
	}
}

uint8_t SPI_GetFlagStatus(st_SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }

    return FLAG_RESET;
}

void SPI_PeripheralControl(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        /* Enable SPI */
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        /* Disable SPI */
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void SPI_SSIConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

void SPI_SSOEConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {   
        /* Enable SS output in master mode */
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        /* Disable SS output in master mode */
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

void SPI_ClearOVRFlag(st_SPI_RegDef_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;
}

void SPI_CloseTransmission(st_SPI_Handle_t *pHandle)
{
    pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pHandle->pTxBuffer = NULL;
    pHandle->TxLen = 0;
    pHandle->TxState = SPI_READY;
}

void SPI_CloseReception(st_SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}

weak void SPI_ApplicationEventCallback(st_SPI_Handle_t *pHandle, uint8_t AppEV)
{
    /* This is a weak implementation. The user application may override this function */
}

static void SPI_TXE_Interrupt_Handle(st_SPI_Handle_t *pHandle)
{
    /* Check DFF bit in CR1 register */
    if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        /* 16 bits DFF, write data from DR register */
        pHandle->pSPIx->DR = *(uint16_t *)pHandle->pTxBuffer;
        pHandle->TxLen--;
        pHandle->TxLen--;
        *(uint16_t *)pHandle->pTxBuffer++;
    }
    else
    {
        /* 8 bits DFF, write data from DR register */
        pHandle->pSPIx->DR = *pHandle->pTxBuffer;
        pHandle->TxLen--;
        *pHandle->pTxBuffer++;
    }

    /* TxLen is 0, close the SPI transmission, inform the application that Tx is over */
    if (!pHandle->TxLen)
    {
        /* Prevent the interrupt */
        SPI_CloseTransmission(pHandle);
        SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void SPI_RXNE_Interrupt_Handle(st_SPI_Handle_t *pHandle)
{
    /* Check DFF bit in CR1 register */
    if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        /* 16 bits DFF, write data from DR register */
        *((uint16_t *)pHandle->pRxBuffer) = (uint16_t)pHandle->pSPIx->DR;
        pHandle->RxLen--;
        pHandle->RxLen--;
        pHandle->pRxBuffer++;
        pHandle->pRxBuffer++;
    }
    else
    {
        /* 8 bits DFF, write data from DR register */
        *pHandle->pRxBuffer = (uint8_t)pHandle->pSPIx->DR;
        pHandle->RxLen--;
        *pHandle->pRxBuffer++;
    }

    /* RxLen is 0, close the SPI transmission, inform the application that Rx is over */
    if (!pHandle->RxLen)
    {
        /* Prevent the interrupt */
        SPI_CloseTransmission(pHandle);
        SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void SPI_OVR_Interrupt_Handle(st_SPI_Handle_t *pHandle)
{
    uint8_t temp;

    /* Clear the OVR flag */
	if(pHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}

    (void)temp;

    /* Inform Application */
    SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);
}