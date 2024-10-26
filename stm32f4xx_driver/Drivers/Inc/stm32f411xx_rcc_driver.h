#ifndef INC_STM32F411XX_RCC_DRIVER_H_
#define INC_STM32F411XX_RCC_DRIVER_H_

#include "stm32f411xx.h"

#define RCC_SYSTEM_CLOCK_STATUS_HSI_OSCILLATOR  0
#define RCC_SYSTEM_CLOCK_STATUS_HSE_OSCILLATOR  1
#define RCC_SYSTEM_CLOCK_STATUS_PLL             2

#define RCC_HSI_CLOCK                           (16000000U)
#define RCC_HSE_CLOCK                           (8000000U)

/*
 * RCC Oscillator Type
 */

#define RCC_OSCILLATORTYPE_NONE                 (0x00000000U)
#define RCC_OSCILLATORTYPE_HSE                  (0x00000001U)
#define RCC_OSCILLATORTYPE_HSI                  (0x00000002U)
#define RCC_OSCILLATORTYPE_LSE                  (0x00000004U)
#define RCC_OSCILLATORTYPE_LSI                  (0x00000005U)

/*
 * RCC HSE Config
 */

#define RCC_HSE_OFF                             (0x00000000U)
#define RCC_HSE_ON
#define RCC_HSE_BYPASS

/*
 * RCC System Clock
 */

#define RCC_SYSTEM_CLOCK_DIV_2                  (0x00000008U)
#define RCC_SYSTEM_CLOCK_DIV_4                  (0x00000009U)
#define RCC_SYSTEM_CLOCK_DIV_8                  (0x0000000AU)
#define RCC_SYSTEM_CLOCK_DIV_16                 (0x0000000BU)
#define RCC_SYSTEM_CLOCK_DIV_64                 (0x0000000CU)
#define RCC_SYSTEM_CLOCK_DIV_128                (0x0000000DU)
#define RCC_SYSTEM_CLOCK_DIV_256                (0x0000000EU)
#define RCC_SYSTEM_CLOCK_DIV_512                (0x0000000FU)

/*
 * RCC AHB Clock
 */

#define RCC_AHB_CLOCK_DIV_2                     (0x00000004U)
#define RCC_AHB_CLOCK_DIV_4                     (0x00000005U)
#define RCC_AHB_CLOCK_DIV_8                     (0x00000006U)
#define RCC_AHB_CLOCK_DIV_16                    (0x00000007U)

/*
 * RCC Main PLL division factor for main system clock
 */

#define RCC_PLLP_DIV_2                          (0x00000000U)
#define RCC_PLLP_DIV_4                          (0x00000001U)
#define RCC_PLLP_DIV_8                          (0x00000002U)
#define RCC_PLLP_DIV_16                         (0x00000003U)



#define RCC_AHB_PRESCALER(x) \
    ((x == RCC_SYSTEM_CLOCK_DIV_2)      ? 2u     : \
     (x == RCC_SYSTEM_CLOCK_DIV_4)      ? 4u     : \
     (x == RCC_SYSTEM_CLOCK_DIV_8)      ? 8u     : \
     (x == RCC_SYSTEM_CLOCK_DIV_16)     ? 16u    : \
     (x == RCC_SYSTEM_CLOCK_DIV_64)     ? 64u    : \
     (x == RCC_SYSTEM_CLOCK_DIV_128)    ? 128u   : \
     (x == RCC_SYSTEM_CLOCK_DIV_256)    ? 256u   : \
     (x == RCC_SYSTEM_CLOCK_DIV_512)    ? 512u   : 0)

#define RCC_APB_PRESCALER(x) \
    ((x == RCC_AHB_CLOCK_DIV_2)         ? 2u     : \
     (x == RCC_AHB_CLOCK_DIV_4)         ? 4u     : \
     (x == RCC_AHB_CLOCK_DIV_8)         ? 8u     : \
     (x == RCC_AHB_CLOCK_DIV_16)        ? 16u    : 0)

#define RCC_PLLP_DIV_FACTOR(x) \
    ((x == RCC_PLLP_DIV_2)  ? 2u : \
     (x == RCC_PLLP_DIV_4)  ? 4u : \
     (x == RCC_PLLP_DIV_8)  ? 6u : \
     (x == RCC_PLLP_DIV_16) ? 8u : 2u)


/*
 * RCC PLL configuration structure definition
 */
typedef struct
{
    uint32_t PLLState;
    uint32_t PLLSource;
    uint32_t PLLM;              /* PLLM : Division factor for the main PLL input clock, 2 <= PLLM <= 63 */
    uint32_t PLLN;              /* PLLN : Main PLL multiplication factor for VCO, 50 <= PLLN <= 432 */
    uint32_t PLLP;              /* PLLP : Main PLL division factor for main system clock, PLLP = 2, 4, 6, 8 */
    uint32_t PLLQ;              /* PLLQ : Main PLL division factor for USB OTG FS, and SDIO clocks, 2 <= PLLQ <= 15 */
} st_RCC_PLLInitTypeDef_t;

uint32_t RCC_GetSystemClock(void);

uint32_t RCC_GetAHBPrescaler(void);

uint32_t RCC_GetAPB1Prescaler(void);
uint32_t RCC_GetAPB1Prescaler(void);

uint32_t RCC_GetAPBLowSpeedPrescaler(void);
uint32_t RCC_GetAPBHighSpeedPrescaler(void);

uint32_t RCC_GetPLLOutputClock(void);



#endif /* INC_STM32F411XX_RCC_DRIVER_H_ */