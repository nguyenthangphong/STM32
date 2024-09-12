#include "stm32f411xx_usart_driver.h"
#include "stm32f411xx_rcc_driver.h"

void USART_PeriClockControl(st_USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }
        else if (pUSARTx == USART6)
        {
            USART6_PCLK_EN();
        }
        else
        {
            /* Do nothing */
        }
    }
}

void USART_Init(st_USART_Handle_t *pUSARTHandle)
{
    /*
     * Configuration of CR1 register
     */

    uint32_t temp = 0;

    /* Enable USART for the Peripheral Clock */
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    /* Enable USART Tx and Rx to the USART Mode configuration item */
    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        temp |= (1 << USART_CR1_RE);
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        temp |= (1 << USART_CR1_TE);
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX_RX)
    {
        temp |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
    }
    else
    {
        /* Do nothing */
    }

    /* Configure the word length configuration item */
    temp |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

    /* Configuration of parity control bit fields */
    if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        /* Enable parity control */
        temp |= (1 << USART_CR1_PCE);
    }
    else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
    {
        /* Enable parity control */
        temp |= (1 << USART_CR1_PCE);

        /* Enable Odd parity */
        temp |= (1 << USART_CR1_PS);
    }
    else
    {
        /* Do nothing */
    }

    /* Set into CR1 register */
    pUSARTHandle->pUSARTx->CR1 = temp;

    /*
     * Configuration of CR2 register
     */

    temp = 0;

    /* Configure the number of stop bits inserted during USART frame transmission */
    temp |= (pUSARTHandle->USART_Config.USART_NoOfStopBit << USART_CR2_STOP);

    /* Set into CR2 register */
    pUSARTHandle->pUSARTx->CR2 = temp;

    /*
     * Configuration of CR3 register
     */

    temp = 0;

    /* Configuration of USART Hardware flow control */
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        /* Enable CTS flow control */
        temp |= (1 << USART_CR3_CTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        /* Enable RTS flow control */
        temp |= (1 << USART_CR3_RTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        /* Enable CTS and RTS flow control */
        temp |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
    }
    else
    {
        /* Do nothing */
    }

    /* Set into CR3 register */
    pUSARTHandle->pUSARTx->CR3 = temp;

    /*
     * Configuration of BRR register
     */

    temp = 0;

    /* Configuration of the baud rate */
    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_SendData(st_USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint16_t *pData;

    /* Loop over until "Len" number of bytes are transferred */
    for (uint32_t i = 0; i < Len; i++)
    {
        /* Wait until TXE flag is set in the SR register */
        while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

        /* Check the USART Word Lengh item for 9 Bits or 8 Bits in a Frame */
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_9BITS)
        {
            /* Load the DR register with 2 bytes masking the bits other than first 9 bits */
            pData = (uint16_t *)pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

            /* Check for USART Parity Control */
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                /* No parity bit is used in this transfer, so 9 bits of user data will be sent */
                pTxBuffer++;
                pTxBuffer++;
            }
            else
            {
                /* Parity bit is used in this transfer, so 8 bits of user data will be sent */
                pTxBuffer++;
            }
        }
        else
        {
            /* 8 bits data transfer */
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

            /* Increment the buffer address */
            pTxBuffer++;
        }
    }

    /* Wail until TC flag is set in the SR register */
    while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

uint8_t USART_GetFlagStatus(st_USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if (pUSARTx->SR & StatusFlagName)
    {
        return SET;
    }

    return RESET;
}

void USART_ClearFlag(st_USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    pUSARTx->SR &= ~(StatusFlagName);
}

void USART_PeripheralControl(st_USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pUSARTx->CR1 |= (1 << USART_CR1_UE);
    }
    else
    {
        pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
    }
}

void USART_SetBaudRate(st_USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    uint32_t PCLKx;
    uint32_t USARTdiv;
    uint32_t Mpart, Fpart;
    uint32_t temp = 0;

    /* Get the value of APB bus clock into the PCLKx */
    if (pUSARTx == USART1 || pUSARTx == USART6)
    {
        /* USART1 and USART6 are hanging on APB2 bus */
        PCLKx = RCC_GetPCLK2Value();
    }
    else
    {
        /* USART2 is hanging on APB1 bus */
        PCLKx = RCC_GetPCLK1Value();
    }

    /* Check for OVER8 configuration bit */
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        /* OVER8 = 1 : Over sampling by 8 */
        USARTdiv = ((25 * PCLKx) / (2 * BaudRate));
    }
    else
    {
        /* Over sampling by 16 */
        USARTdiv = ((25 * PCLKx) / (4 * BaudRate));
    }

    /* Calculate the Mantissa part */
    Mpart = USARTdiv / 100;

    /* Place the Mantissa part in appropriate bit possition */
    temp |= (Mpart << USART_BRR_DIV_MANTISSA);

    /* Extract the Fraction part */
    Fpart = (USARTdiv - (Mpart * 100));

    /* Calculate the Final Fractional */
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        /* OVER8 = 1 : Over sampling by 8 */
        Fpart = (((Fpart * 8) + 50) / 100) & ((uint8_t)0x7);
    }
    else
    {
        /* Over sampling by 16 */
        Fpart = (((Fpart * 16) + 50) / 100) & ((uint8_t)0xF);
    }

    /* Place the Fractional part in appropriate bit position */
    temp |= Fpart;

    /* Set into BRR register */
    pUSARTx->BRR = temp;
}