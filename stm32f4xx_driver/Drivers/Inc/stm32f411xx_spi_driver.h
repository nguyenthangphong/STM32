#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */

typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BugConfig;
    uint8_t SPI_SCLKSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} st_SPI_Config_t;

/*
 *  Handle structure for SPIx peripheral
 */

typedef struct
{
    st_SPI_RegDef_t         *pSPIx;
    st_SPI_Config_t         SPI_Config;
} st_SPI_Handle_t;

/*
 * Peripheral Clock Setup
 */

void SPI_PeriClockControl(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De Init
 */

void SPI_Init(st_SPI_Handle_t *pSPIHandle);
void SPI_DeInit(st_SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */

void SPI_SendData(st_SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(st_SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);

/*
 * IRQ Configuration and ISR Handling
 */

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(st_SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */

uint8_t SPI_GetFlagStatus(st_SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */