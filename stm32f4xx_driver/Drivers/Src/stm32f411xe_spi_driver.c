#include "stm32f411xe_spi_driver.h"
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

e_StatusTypeDef_t SPI_Init(st_SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle == NULL)
    {
        return STATUS_ERROR;
    }

    /* Enable SPI for the Peripheral Clock */
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    /* Configure the Device Mode */
    pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_MSTR)) | ((pSPIHandle->SPI_Config.SPI_MSTR << SPI_CR1_MSTR_POS) & SPI_CR1_MSTR);

    /* Full duplex mode */
    if (pSPIHandle->SPI_Config.SPI_BIDIMODE_RXONLY == SPI_BIDIMODE_RXONLY_FULLDUPLEX)
    {
        pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_BIDIMODE)) | ((SPI_BIDIMODE_2_LINE << SPI_CR1_BIDIMODE_POS) & SPI_CR1_BIDIMODE);
        pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_RXONLY)) | ((SPI_RXONLY_TX_RX << SPI_CR1_RXONLY_POS) & SPI_CR1_RXONLY);
    }
    /* Half duplex mode */
    else if (pSPIHandle->SPI_Config.SPI_BIDIMODE_RXONLY == SPI_BIDIMODE_RXONLY_HALFDUPLEX)
    {
        pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_BIDIMODE)) | ((SPI_BIDIMODE_1_LINE << SPI_CR1_BIDIMODE_POS) & SPI_CR1_BIDIMODE);
        pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_RXONLY)) | ((SPI_RXONLY_TX_RX << SPI_CR1_RXONLY_POS) & SPI_CR1_RXONLY);
    }
    /* Simplex mode */
    else if (pSPIHandle->SPI_Config.SPI_BIDIMODE_RXONLY == SPI_BIDIMODE_RXONLY_SIMPLEX_RXONLY)
    {
        pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_BIDIMODE)) | ((SPI_BIDIMODE_1_LINE << SPI_CR1_BIDIMODE_POS) & SPI_CR1_BIDIMODE);
        pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_RXONLY)) | ((SPI_RXONLY_RX << SPI_CR1_RXONLY_POS) & SPI_CR1_RXONLY);
    }
    else
    {
        /* Do nothing */
    }

    /* Configure the SCLK Speed */
    pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_BR)) | ((pSPIHandle->SPI_Config.SPI_BR << SPI_CR1_BR_POS) & SPI_CR1_BR);

    /* Configure the DFF */
    pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_DFF)) | ((pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF_POS) & SPI_CR1_DFF);

    /* Configure the CPOL */
    pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_CPOL)) | ((pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL_POS) & SPI_CR1_CPOL);

    /* Configure the CPHA */
    pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_CPHA)) | ((pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA_POS) & SPI_CR1_CPHA);

    /* Configure the SSM */
    pSPIHandle->pSPIx->CR1 = (pSPIHandle->pSPIx->CR1 & ~(SPI_CR1_SSM)) | ((pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM_POS) & SPI_CR1_SSM);

    return STATUS_OK;
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
        while (((pSPIx->SR & SPI_SR_TXE) >> SPI_SR_TXE_POS) == SPI_TXE_NOT_EMPTY)
        {
            /* Wait until the Tx buffer is empty */
        }

        /* 16 bits data frame format */
        if (((pSPIx->CR1 & SPI_CR1_DFF) >> SPI_CR1_DFF_POS) == SPI_DFF_16_BITS_DATA)
        {
            pSPIx->DR = *(uint16_t *)pTxBuffer;
            length--;
            length--;
            (uint16_t *)pTxBuffer++;
        }
        /* 8 bits data frame format */
        else
        {
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
        while (((pSPIx->SR & SPI_SR_RXNE) >> SPI_SR_RXNE_POS) == SPI_RXNE_EMPTY)
        {
            /* Wait until the Rx buffer is not empty */
        }

        /* 16 bits data frame format */
        if (((pSPIx->CR1 & SPI_CR1_DFF) >> SPI_CR1_DFF_POS) == SPI_DFF_16_BITS_DATA)
        {
            *((uint16_t *)pRxBuffer) = pSPIx->DR;
            length--;
            length--;
            (uint16_t *)pRxBuffer++;
        }
        /* 8 bits data frame format */
        else
        {
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
        pHandle->pSPIx->CR2 = (pHandle->pSPIx->CR2 & ~(SPI_CR2_TXEIE)) | ((SET << SPI_CR2_TXEIE_POS) & SPI_CR2_TXEIE);
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
        pHandle->pSPIx->CR2 = (pHandle->pSPIx->CR2 & ~(SPI_CR2_RXNEIE)) | ((SET << SPI_CR2_RXNEIE_POS) & SPI_CR2_RXNEIE);
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
	if((((pHandle->pSPIx->SR & SPI_SR_TXE) >> SPI_SR_TXE_POS) == SPI_TXE_EMPTY) && \
       (((pHandle->pSPIx->CR2 & SPI_CR2_TXEIE) >> SPI_CR2_TXEIE_POS) == SPI_TXEIE_NOT_MASKED)
    )
	{
		/* Handle TXE error */
		SPI_TXE_Interrupt_Handle(pHandle);
	}

    /* Check the RXNE flag */
	if((((pHandle->pSPIx->SR & SPI_SR_RXNE) >> SPI_SR_RXNE_POS) == SPI_RXNE_NOT_EMPTY) && \
       (((pHandle->pSPIx->CR2 & SPI_CR2_RXNEIE) >> SPI_CR2_RXNEIE_POS) == SPI_RXNEIE_NOT_MASKED)
    )
	{
		/* Handle RXNE error */
		SPI_RXNE_Interrupt_Handle(pHandle);
	}

	/* Check the OVR flag */
	if((((pHandle->pSPIx->SR & SPI_SR_OVR) >> SPI_SR_OVR_POS) == SPI_OVR_OVERRUN) && \
       (((pHandle->pSPIx->CR2 & SPI_CR2_ERRIE) >> SPI_CR2_ERRIE_POS) == SPI_ERRIE_ENABLED)
    )
	{
		/* Handle OVR error */
		SPI_OVR_Interrupt_Handle(pHandle);
	}
}

void SPI_SPEConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR1 = (pSPIx->CR1 & ~(SPI_CR1_SPE)) | ((SPI_SPE_EN << SPI_CR1_SPE_POS) & SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 = (pSPIx->CR1 & ~(SPI_CR1_SPE)) | ((SPI_SPE_DI << SPI_CR1_SPE_POS) & SPI_CR1_SPE);
    }
}

void SPI_SSIConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR1 = (pSPIx->CR1 & ~(SPI_CR1_SSI)) | ((SPI_SSI_EN << SPI_CR1_SSI_POS) & SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 = (pSPIx->CR1 & ~(SPI_CR1_SSI)) | ((SPI_SSI_DI << SPI_CR1_SSI_POS) & SPI_CR1_SSI);
    }
}

void SPI_SSOEConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {   
        pSPIx->CR2 = (pSPIx->CR2 & ~(SPI_CR2_SSOE)) | ((SPI_SSOE_EN << SPI_CR2_SSOE_POS) & SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 = (pSPIx->CR2 & ~(SPI_CR2_SSOE)) | ((SPI_SSOE_DI << SPI_CR2_SSOE_POS) & SPI_CR2_SSOE);
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
    pHandle->pSPIx->CR2 = (pHandle->pSPIx->CR2 & ~(SPI_CR2_TXEIE)) | ((SPI_TXEIE_NOT_MASKED << SPI_CR2_TXEIE_POS) & SPI_CR2_TXEIE);
    pHandle->pTxBuffer = NULL;
    pHandle->TxLen = 0;
    pHandle->TxState = SPI_READY;
}

void SPI_CloseReception(st_SPI_Handle_t *pHandle)
{
    pHandle->pSPIx->CR2 = (pHandle->pSPIx->CR2 & ~(SPI_CR2_RXNEIE)) | ((SPI_RXNEIE_NOT_MASKED << SPI_CR2_RXNEIE_POS) & SPI_CR2_RXNEIE);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}

weak void SPI_ApplicationEventCallback(st_SPI_Handle_t *pHandle, uint8_t AppEV)
{
    /* This is a weak implementation. The user application may override this function */
}

void SPI_TXE_Interrupt_Handle(st_SPI_Handle_t *pHandle)
{
    /* Check DFF bit in CR1 register */
    if(((pHandle->pSPIx->CR1 & SPI_CR1_DFF) >> SPI_CR1_DFF_POS))
    {
        /* 16 bits DFF, write data from DR register */
        pHandle->pSPIx->DR = *(uint16_t *)pHandle->pTxBuffer;
        pHandle->TxLen--;
        pHandle->TxLen--;
        (uint16_t *)pHandle->pTxBuffer++;
    }
    else
    {
        /* 8 bits DFF, write data from DR register */
        pHandle->pSPIx->DR = *pHandle->pTxBuffer;
        pHandle->TxLen--;
        pHandle->pTxBuffer++;
    }

    /* TxLen is 0, close the SPI transmission, inform the application that Tx is over */
    if (!pHandle->TxLen)
    {
        /* Prevent the interrupt */
        SPI_CloseTransmission(pHandle);
        SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
    }
}

void SPI_RXNE_Interrupt_Handle(st_SPI_Handle_t *pHandle)
{
    /* Check DFF bit in CR1 register */
    if(((pHandle->pSPIx->CR1 & SPI_CR1_DFF) >> SPI_CR1_DFF_POS))
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
        pHandle->pRxBuffer++;
    }

    /* RxLen is 0, close the SPI transmission, inform the application that Rx is over */
    if (!pHandle->RxLen)
    {
        /* Prevent the interrupt */
        SPI_CloseTransmission(pHandle);
        SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
    }
}

void SPI_OVR_Interrupt_Handle(st_SPI_Handle_t *pHandle)
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