#ifndef INC_STM32F411XE_SPI_DRIVER_H_
#define INC_STM32F411XE_SPI_DRIVER_H_

#include "stm32f411xe.h"

/* Configuration structure for SPIx peripheral */
typedef struct
{
    uint8_t SPI_MSTR;
    uint8_t SPI_BIDIMODE_RXONLY;
    uint8_t SPI_BR;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} st_SPI_Config_t;

/* Handle structure for SPIx peripheral */
typedef struct
{
    st_SPI_RegDef_t         *pSPIx;
    st_SPI_Config_t         SPI_Config;
    uint8_t                 *pTxBuffer;
    uint8_t                 *pRxBuffer;
    uint32_t                TxLen;
    uint32_t                RxLen;
    uint8_t                 TxState;
    uint8_t                 RxState;
} st_SPI_Handle_t;

/**************************************** Configuration CR1 register ****************************************/

/* Clock phase */
#define SPI_CPHA_LOW                    (0x00000000U)           /*!< The first clock transition is the first data capture edge */
#define SPI_CPHA_HIGH                   (0x00000001U)           /*!< The second clock transition is the first data capture edge */

/* Clock polarity */
#define SPI_CPOL_LOW                    (0x00000000U)           /*!< CK to 0 when idle */
#define SPI_CPOL_HIGH                   (0x00000001U)           /*!< CK to 1 when idle */

/* Master selection */
#define SPI_MSTR_SLAVE                  (0x00000000U)           /*!< Slave configuration */
#define SPI_MSTR_MASTER                 (0x00000001U)           /*!< Master configuration */

/* Baud rate control */
#define SPI_BR_DIV_2                    SPI_CR1_BR_DIV_2        /*!< f_PCLK / 2 */
#define SPI_BR_DIV_4                    SPI_CR1_BR_DIV_4        /*!< f_PCLK / 4 */
#define SPI_BR_DIV_8                    SPI_CR1_BR_DIV_8        /*!< f_PCLK / 8 */
#define SPI_BR_DIV_16                   SPI_CR1_BR_DIV_16       /*!< f_PCLK / 16 */
#define SPI_BR_DIV_32                   SPI_CR1_BR_DIV_32       /*!< f_PCLK / 32 */
#define SPI_BR_DIV_64                   SPI_CR1_BR_DIV_64       /*!< f_PCLK / 64 */
#define SPI_BR_DIV_128                  SPI_CR1_BR_DIV_128      /*!< f_PCLK / 128 */
#define SPI_BR_DIV_256                  SPI_CR1_BR_DIV_256      /*!< f_PCLK / 256 */

/* SPI enable */
#define SPI_SPE_DI                      (0x00000000U)           /*!< Peripheral disabled */
#define SPI_SPE_EN                      (0x00000001U)           /*!< Peripheral enabled */

/* Frame format */
#define SPI_LSBFIRST_MSB                (0x00000000U)           /*!< MSB transmitted first */
#define SPI_LSBFIRST_LSB                (0x00000001U)           /*!< LSB transmitted first */

/* Internal slave select */
#define SPI_SSI_DI                      (0x00000000U)           /*!< SSI disabled */
#define SPI_SSI_EN                      (0x00000001U)           /*!< SSI enabled only when SSM enabled */

/* Software slave management */
#define SPI_SSM_DI                      (0x00000000U)           /*!< Software slave management disabled */
#define SPI_SSM_EN                      (0x00000001U)           /*!< Software slave management enabled */

/* Receive only */
#define SPI_RXONLY_TX_RX                (0x00000000U)           /*!< Full duplex (Transmit and receive) */
#define SPI_RXONLY_RX                   (0x00000001U)           /*!< Output disabled (Receive-only mode) */

/* Data frame format */
#define SPI_DFF_8_BITS_DATA             (0x00000000U)           /*!< 8 bits data format */
#define SPI_DFF_16_BITS_DATA            (0x00000001U)           /*!< 16 bits data format */

/* CRC transfer next */
#define SPI_CRC_DATA_PHASE              (0x00000000U)           /*!< Data phase (no CRC phase) */
#define SPI_CRC_NEXT_TRANSFER           (0x00000001U)           /*!< Next transfer is CRC (CRC phase) */

/* Hardware CRC calculation enable */
#define SPI_CRCEN_DI                    (0x00000000U)           /*!< CRC calculation disabled */
#define SPI_CRCEN_EN                    (0x00000001U)           /*!< CRC calculation enabled */

/* Output enable in bidirectional mode */
#define SPI_BIDIOE_DI                   (0x00000000U)           /*!< Output disabled (receive-only mode) */
#define SPI_BIDIOE_EN                   (0x00000001U)           /*!< Output enabled (transmit-only mode) */

/* Bidirectional data mode enable */
#define SPI_BIDIMODE_2_LINE             (0x00000000U)           /*!< 2-line unidirectional data mode selected */
#define SPI_BIDIMODE_1_LINE             (0x00000001U)           /*!< 1-line bidirectional data mode selected */

/**************************************** Configuration CR2 register ****************************************/

/* Rx buffer DMA enable */
#define SPI_RXDMAEN_DI                  (0x00000000U)           /*!< Rx buffer DMA disabled */
#define SPI_RXDMAEN_EN                  (0x00000001U)           /*!< Rx buffer DMA enabled */

/* Tx buffer DMA enable */
#define SPI_TXDMAEN_DI                  (0x00000000U)           /*!< Tx buffer DMA disabled */
#define SPI_TXDMAEN_EN                  (0x00000001U)           /*!< Tx buffer DMA enabled */

/* SS output enable */
#define SPI_SSOE_DI                     (0x00000000U)           /*!< SS output is disabled in master mode and the cell can work in multimaster configuration */
#define SPI_SSOE_EN                     (0x00000001U)           /*!< SS output is enabled in master mode and when the cell is enabled. The cell cannot work in a multimaster environment. */

/* Frame format */
#define SPI_FRF_MOTOROLA_MODE           (0x00000000U)           /*!< SPI Motorola mode */
#define SPI_FRF_TI_MODE                 (0x00000001U)           /*!< SPI TI mode */

/* Error interrupt enable */
#define SPI_ERRIE_MASKED                (0x00000000U)           /*!< Error interrupt is masked */
#define SPI_ERRIE_ENABLED               (0x00000001U)           /*!< Error interrupt is enabled */

/* RX buffer not empty interrupt enable */
#define SPI_RXNEIE_MASKED               (0x00000000U)           /*!< RXNE interrupt masked */
#define SPI_RXNEIE_NOT_MASKED           (0x00000001U)           /*!< RXNE interrupt not masked */

/* Tx buffer empty interrupt enable */
#define SPI_TXEIE_MASKED                (0x00000000U)           /*!< TXE interrupt masked */
#define SPI_TXEIE_NOT_MASKED            (0x00000001U)           /*!< TXE interrupt not masked */

/**************************************** Configuration SR register ****************************************/

/* Receive buffer not empty */
#define SPI_RXNE_EMPTY                  (0x00000000U)           /*!< Rx buffer empty */
#define SPI_RXNE_NOT_EMPTY              (0x00000001U)           /*!< Rx buffer not empty */

/* Transmit buffer empty */
#define SPI_TXE_NOT_EMPTY               (0x00000000U)           /*!< Tx buffer not empty */
#define SPI_TXE_EMPTY                   (0x00000001U)           /*!< Tx buffer empty */

/* Channel side */
#define SPI_CHSIDE_LEFT                 (0x00000000U)           /*!< Channel Left has to be transmitted or has been received */
#define SPI_CHSIDE_RIGHT                (0x00000001U)           /*!< Channel Right has to be transmitted or has been received */

/* Underrun flag */
#define SPI_UDR_NO_UNDERRUN             (0x00000000U)           /*!< No underrun occurred */
#define SPI_UDR_UNDERRUN                (0x00000001U)           /*!< Underrun occurred */

/* CRC error flag */
#define SPI_CRCERR_MATCH                (0x00000000U)           /*!< CRC value received matches the SPI_RXCRCR value */
#define SPI_CRCERR_NOT_MATCH            (0x00000001U)           /*!< CRC value received does not match the SPI_RXCRCR value */

/* Mode fault */
#define SPI_MODF_NO_MODE_FAULT          (0x00000000U)           /*!< No mode fault occurred */
#define SPI_MODF_MODE_FAULT             (0x00000001U)           /*!< Mode fault occurred */

/* Overrun flag */
#define SPI_OVR_NO_OVERRUN              (0x00000000U)           /*!< No overrun occurred */
#define SPI_OVR_OVERRUN                 (0x00000001U)           /*!< Overrun occurred */

/* Busy flag */
#define SPI_BSY_NOT_BUSY                (0x00000000U)           /*!< SPI(or I2S) not busy */
#define SPI_BSY_BUSY                    (0x00000001U)           /*!< SPI(or I2S) is busy in communication or Tx buffer is not empty */

/* Frame format error */
#define SPI_FRE_NO_ERROR                (0x00000000U)           /*!< No frame format error */
#define SPI_FRE_ERROR                   (0x00000001U)           /*!< A frame format error occurred */

/**************************************** Configuration I2SCFGR register ****************************************/

/**************************************** Configuration I2SPR register ****************************************/

/* Bidirectional data mode */
#define SPI_BIDIMODE_RXONLY_FULLDUPLEX       (SPI_BIDIMODE_2_LINE | SPI_RXONLY_TX_RX)
#define SPI_BIDIMODE_RXONLY_HALFDUPLEX       (SPI_BIDIMODE_1_LINE | SPI_RXONLY_TX_RX)
#define SPI_BIDIMODE_RXONLY_SIMPLEX_RXONLY   (SPI_BIDIMODE_1_LINE | SPI_RXONLY_RX)

/* Application state */
#define SPI_READY			            0
#define SPI_BUSY_IN_RX		            1
#define SPI_BUSY_IN_TX		            2

/* SPI Application States */
#define SPI_EVENT_TX_CMPLT              1
#define SPI_EVENT_RX_CMPLT              2
#define SPI_EVENT_OVR_ERR	            3
#define SPI_EVENT_CRC_ERR	            4

void SPI_PeriClockControl(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);
e_StatusTypeDef_t SPI_Init(st_SPI_Handle_t *pSPIHandle);
void SPI_DeInit(st_SPI_RegDef_t *pSPIx);
void SPI_SendData(st_SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(st_SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);
uint8_t SPI_SendDataIT(st_SPI_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_ReceiveDataIT(st_SPI_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t length);
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(st_SPI_Handle_t *pHandle);
void SPI_TXE_Interrupt_Handle(st_SPI_Handle_t *pHandle);
void SPI_RXNE_Interrupt_Handle(st_SPI_Handle_t *pHandle);
void SPI_OVR_Interrupt_Handle(st_SPI_Handle_t *pHandle);
void SPI_SPEConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(st_SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(st_SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(st_SPI_Handle_t *pHandle);
void SPI_CloseReception(st_SPI_Handle_t *pHandle);
void SPI_ApplicationEventCallback(st_SPI_Handle_t *pSandle, uint8_t AppEV);

#endif /* INC_STM32F411XE_SPI_DRIVER_H_ */