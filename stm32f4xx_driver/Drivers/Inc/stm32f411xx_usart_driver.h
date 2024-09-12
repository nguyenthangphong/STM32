#ifndef INC_STM32F411XX_USART_DRIVER_H_
#define INC_STM32F411XX_USART_DRIVER_H_

#include "stm32f411xx.h"

/*
 *  Configuration structure for USARTx peripheral
 */

typedef struct
{
    uint8_t USART_Mode;
    uint8_t USART_Baud;
    uint8_t USART_NoOfStopBit;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;
} st_USART_Config_t;

/*
 *  Handle structure for USARTx peripheral
 */

typedef struct
{
    st_USART_RegDef_t                   *pUSARTx;
    st_USART_Config_t                   USART_Config;
    uint8_t                             *pTxBuffer;
    uint8_t                             *pRxBuffer;
    uint32_t                            TxLen;
    uint32_t                            RxLen;
    uint8_t                             TxBusyState;
    uint8_t                             RxBusyState;
} st_USART_Handle_t;

/*
 *  USART Mode
 */

#define USART_MODE_ONLY_TX 	            0
#define USART_MODE_ONLY_RX 	            1
#define USART_MODE_TX_RX  	            2

/*
 *  USART Baud
 */

#define USART_BAUD_1200				    1200
#define USART_BAUD_2400				    2400
#define USART_BAUD_9600				    9600
#define USART_BAUD_19200 			    19200
#define USART_BAUD_38400 			    38400
#define USART_BAUD_57600 			    57600
#define USART_BAUD_115200 			    115200
#define USART_BAUD_230400 			    230400
#define USART_BAUD_460800 			    460800
#define USART_BAUD_921600 			    921600
#define USART_BAUD_2M 				    2000000
#define USART_BAUD_3M 				    3000000

/*
 *  USART No of Stop Bit
 */

#define USART_STOP_BITS_1               0
#define USART_STOP_BITS_0_5             1
#define USART_STOP_BITS_2               2
#define USART_STOP_BITS_1_5             3

/*
 *  USART Word Length
 */

#define USART_WORD_LENGTH_8BITS         0
#define USART_WORD_LENGTH_9BITS         1

/*
 *  USART Parity Control
 */

#define USART_PARITY_DISABLE            0
#define USART_PARITY_EN_EVEN            1
#define USART_PARITY_EN_ODD             2

/*
 *  USART HW Flow Control
 */

#define USART_HW_FLOW_CTRL_NONE    	    0
#define USART_HW_FLOW_CTRL_CTS    	    1
#define USART_HW_FLOW_CTRL_RTS    	    2
#define USART_HW_FLOW_CTRL_CTS_RTS	    3

/*
 *  USART related state flags definitions
 */

#define USART_FLAG_TXE 			        (1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		        (1 << USART_SR_RXNE)
#define USART_FLAG_TC 			        (1 << USART_SR_TC)

/*
 *  USART related state apps definitions
 */

#define USART_READY 		            0
#define USART_BUSY_IN_RX 	            1
#define USART_BUSY_IN_TX 	            2

/*
 *  USART related event errors definitions
 */

#define USART_EVENT_TX_CMPLT   	        0
#define	USART_EVENT_RX_CMPLT   	        1
#define	USART_EVENT_IDLE      	        2
#define	USART_EVENT_CTS       	        3
#define	USART_EVENT_PE        	        4
#define	USART_ERR_FE     		        5
#define	USART_ERR_NE    	 	        6
#define	USART_ERR_ORE    		        7

/*
 * Peripheral Clock setup
 */

void USART_PeriClockControl(st_USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and De-init
 */

void USART_Init(st_USART_Handle_t *pUSARTHandle);
void USART_DeInit(st_USART_Handle_t *pUSARTHandle);

/*
 * Data Send and Receive
 */

void USART_SendData(st_USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void  USART_ReceiveData(st_USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(st_USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(st_USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(st_USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */

uint8_t USART_GetFlagStatus(st_USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(st_USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(st_USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(st_USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callbacks
 */

void USART_ApplicationEventCallback(st_USART_Handle_t *pUSARTHandle, uint8_t ApEv);

#endif /* INC_STM32F411XX_USART_DRIVER_H_ */