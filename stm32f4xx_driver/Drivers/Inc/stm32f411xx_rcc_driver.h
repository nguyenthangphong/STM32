#ifndef INC_STM32F411XX_RCC_DRIVER_H_
#define INC_STM32F411XX_RCC_DRIVER_H_

#include "stm32f411xx.h"

#define RCC_SYSTEM_CLOCK_STATUS_HSI_OSCILLATOR  0
#define RCC_SYSTEM_CLOCK_STATUS_HSE_OSCILLATOR  1
#define RCC_SYSTEM_CLOCK_STATUS_PLL             2

#define RCC_HSI_CLOCK                           (16000000U)
#define RCC_HSE_CLOCK                           (8000000U)

uint32_t RCC_GetPCLK1Value(void);

uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F411XX_RCC_DRIVER_H_ */