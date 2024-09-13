#include "stm32f411xx_spi_driver.h"

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
    pSPIHandle->pSPIx->CR1 = temp;;
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
        if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            /* 16 bits DFF, load data into DR register */
            pSPIx->DR = *(uint16_t *)pTxBuffer;
            length--;
            length--;
            (uint16_t *)pTxBuffer++;
        }
        else
        {
            /* 8 bits DFF, load data into DR register */
            pSPIx->DR = *pTxBuffer;
            length--;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveData(st_SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{

}

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandling(st_SPI_Handle_t *pHandle)
{

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