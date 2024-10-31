#ifndef INC_STM32F411XE_GPIO_DRIVER_H_
#define INC_STM32F411XE_GPIO_DRIVER_H_

#include "stm32f411xe.h"

/*
 * Configuration Structure for GPIO Pin
 */

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
} st_GPIO_PinConfig_t;

/*
 * Handle Structure for GPIO Pin
 */

typedef struct
{
    st_GPIO_RegDef_t               *pGPIOx;
    st_GPIO_PinConfig_t            GPIO_PinConfig;
} st_GPIO_Handle_t;

/*
 * GPIO Pin Numbers
 */

#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

/*
 * GPIO Pin Possible Modes
 */

#define GPIO_MODE_IN            0
#define GPIO_MODE_OUT           1
#define GPIO_MODE_ALTFN         2
#define GPIO_MODE_ANALOG        3
#define GPIO_MODE_IT_FT         4
#define GPIO_MODE_IT_RT         5
#define GPIO_MODE_IT_RFT        6

/*
 * GPIO Pin Possible Output Types
 */

#define GPIO_OP_TYPE_PP         0
#define GPIO_OP_TYPE_OD         1

/*
 * GPIO Pin Possible Output Speeds
 */

#define GPIO_SPEED_LOW          0
#define GPIO_SPEED_MEDIUM       1
#define GPIO_SPEED_FAST         2
#define GPIO_SPEED_HIGH         3

/*
 * GPIO Pin Pull-Up and Pull-Down Configuration Macros
 */

#define GPIO_NO_PUPD            0
#define GPIO_PU                 1
#define GPIO_PD                 2

/*
 * GPIO Pin Alternate Function Macros
 */

#define GPIO_AF0_MCO            0
#define GPIO_AF1_TIM1           1
#define GPIO_AF1_TIM2           1
#define GPIO_AF2_TIM3           2
#define GPIO_AF2_TIM4           2
#define GPIO_AF2_TIM5           2
#define GPIO_AF3_TIM9           3
#define GPIO_AF3_TIM10          3
#define GPIO_AF3_TIM11          3
#define GPIO_AF4_I2C1           4
#define GPIO_AF4_I2C2           4
#define GPIO_AF4_I2C3           4
#define GPIO_AF5_SPI1           5
#define GPIO_AF5_SPI2           5
#define GPIO_AF5_SPI3           5
#define GPIO_AF5_SPI4           5
#define GPIO_AF6_SPI3           6
#define GPIO_AF6_SPI4           6
#define GPIO_AF6_SPI5           6
#define GPIO_AF7_USART1         7
#define GPIO_AF7_USART2         7
#define GPIO_AF8_USART6         8
#define GPIO_AF9_I2C2           9
#define GPIO_AF9_I2C3           9
#define GPIO_AF10_OTG_FS        10
#define GPIO_AF12_SDIO          12
#define GPIO_AF15_EVENTOUT      15

/*
 * Peripheral Clock Setup
 */

void GPIO_PeriClockControl(st_GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De Init
 */

void GPIO_Init(st_GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(st_GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(st_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(st_GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(st_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(st_GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(st_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F411XE_GPIO_DRIVER_H_ */