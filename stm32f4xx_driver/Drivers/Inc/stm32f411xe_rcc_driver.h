#ifndef INC_STM32F411XE_RCC_DRIVER_H_
#define INC_STM32F411XE_RCC_DRIVER_H_

#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_def.h"

#define RCC_HSI_CLOCK                           (16000000U)
#define RCC_HSE_CLOCK                           (8000000U)

/*
 * RCC Oscillator Type
 */

#define RCC_OSCILLATORTYPE_NONE                 (0x00000000U)
#define RCC_OSCILLATORTYPE_HSE                  (0x00000001U)
#define RCC_OSCILLATORTYPE_HSI                  (0x00000002U)
#define RCC_OSCILLATORTYPE_LSE                  (0x00000004U)
#define RCC_OSCILLATORTYPE_LSI                  (0x00000008U)

/*
 * RCC System Clock Type
 */

#define RCC_CLOCKTYPE_SYSCLK                    (0x00000001U)
#define RCC_CLOCKTYPE_HCLK                      (0x00000002U)
#define RCC_CLOCKTYPE_PCLK1                     (0x00000004U)
#define RCC_CLOCKTYPE_PCLK2                     (0x00000008U)

/*
 * RCC HSE Config
 */

#define RCC_HSE_OFF                             (0x00000000U)
#define RCC_HSE_ON                              (0x00000001U)
#define RCC_HSE_BYPASS                          (0x00000002U)

/*
 * RCC HSI Config
 */

#define RCC_HSI_OFF                             (0x00000000U)
#define RCC_HSI_ON                              (0x00000001U)

#define RCC_HSICALIBRATION_DEFAULT              (0x00000010U)

/*
 * RCC LSE Config
 */

#define RCC_LSE_OFF                             (0x00000000U)
#define RCC_LSE_ON                              (0x00000001U)
#define RCC_LSE_BYPASS                          (0x00000002U)

/*
 * RCC LSI Config
 */

#define RCC_LSI_OFF                             (0x00000000U)
#define RCC_LSI_ON                              (0x00000001U)

/*
 * RCC PLL Config
 */

#define RCC_PLL_NONE                            (0x00000000U)
#define RCC_PLL_OFF                             (0x00000001U)
#define RCC_PLL_ON                              (0x00000002U)

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

/*
 * RCC Microcontroller clock output 1
 */

#define RCC_MCO1_HSI_CLOCK_SOURCE               (0x00000000U)
#define RCC_MCO1_LSE_CLOCK_SOURCE               (0x00000001U)
#define RCC_MCO1_HSE_CLOCK_SOURCE               (0x00000002U)
#define RCC_MCO1_PLL_CLOCK_SOURCE               (0x00000003U)

/*
 * RCC Microcontroller clock output 1 MCO1 prescaler
 */

#define RCC_MCO1_PRESCALER_DIV_2                (0x00000004U)
#define RCC_MCO1_PRESCALER_DIV_3                (0x00000005U)
#define RCC_MCO1_PRESCALER_DIV_4                (0x00000006U)
#define RCC_MCO1_PRESCALER_DIV_5                (0x00000007U)

/*
 * RCC Microcontroller clock output 2
 */

#define RCC_MCO2_SYSTEM_CLOCK_SOURCE            (0x00000000U)
#define RCC_MCO2_PLLI2S_CLOCK_SOURCE            (0x00000001U)
#define RCC_MCO2_HSEOSCILLATOR_CLOCK_SOURCE     (0x00000002U)
#define RCC_MCO2_PLL_CLOCK_SOURCE               (0x00000003U)

/*
 * RCC System clock switch status
 */

#define RCC_SWS_HSI_OSCILLATOR                  (0x00000000U)
#define RCC_SWS_HSE_OSCILLATOR                  (0x00000001U)
#define RCC_SWS_PLL                             (0x00000002U)

/*
 * RCC System clock switch
 */

#define RCC_SW_HSI_OSCILLATOR                   (0x00000000U)
#define RCC_SW_HSE_OSCILLATOR                   (0x00000001U)
#define RCC_SW_PLL                              (0x00000002U)

/*
 * RCC PLL clock source
 */

#define RCC_PLLSRC_HSI                          (0x00000000U)
#define RCC_PLLSRC_HSE                          (0x00000001U)

/*
 * RCC MCO Index
 */

#define RCC_MCO1                                (0x00000000U)
#define RCC_MCO2                                (0x00000001U)

/*
 * RCC Flags in the CR register
 */

#define RCC_FLAG_HSIRDY                         ((uint8_t)0x21U)
#define RCC_FLAG_HSERDY                         ((uint8_t)0x31U)
#define RCC_FLAG_PLLRDY                         ((uint8_t)0x39U)
#define RCC_FLAG_PLLI2SRDY                      ((uint8_t)0x3BU)

/*
 * RCC Flags in the BDCR register
 */

#define RCC_FLAG_LSERDY                         ((uint8_t)0x41U)

/*
 * RCC Flags in the CSR register
 */

#define RCC_FLAG_LSIRDY                         ((uint8_t)0x61U)
#define RCC_FLAG_BORRST                         ((uint8_t)0x79U)
#define RCC_FLAG_PINRST                         ((uint8_t)0x7AU)
#define RCC_FLAG_PORRST                         ((uint8_t)0x7BU)
#define RCC_FLAG_SFTRST                         ((uint8_t)0x7CU)
#define RCC_FLAG_IWDGRST                        ((uint8_t)0x7DU)
#define RCC_FLAG_WWDGRST                        ((uint8_t)0x7EU)
#define RCC_FLAG_LPWRRST                        ((uint8_t)0x7FU)

/*
 * RCC registers bit address in the alias region
 */

#define RCC_OFFSET                              (RCC_BASEADDR - PERIPH_BASEADDR)

/* CR Register */
#define RCC_CR_OFFSET                           (RCC_OFFSET + 0x00u)

/* BDCR Register */
#define RCC_BDCR_OFFSET                         (RCC_OFFSET + 0x70u)

/* CSR Register */
#define RCC_CSR_OFFSET                          (RCC_OFFSET + 0x74u)

/* Alias word address of HSION bit */
#define RCC_HSION_BIT_NUMBER                    (0u)
#define RCC_CR_HSION_BB                         (PERIPH_BB_BASEADDR + (RCC_CR_OFFSET * 32u) + (RCC_HSION_BIT_NUMBER * 4u))

/* Alias word address of PLLON bit */
#define RCC_PLLON_BIT_NUMBER                    (24u)
#define RCC_CR_PLLON_BB                         (PERIPH_BB_BASEADDR + (RCC_CR_OFFSET * 32u) + (RCC_PLLON_BIT_NUMBER * 4u))

/* Alias word address of LSEON bit */
#define RCC_LSEON_BIT_NUMBER                    (0u)
#define RCC_BDCR_LSEON_BB                       (PERIPH_BB_BASEADDR + (RCC_BDCR_OFFSET * 32u) + (RCC_LSEON_BIT_NUMBER * 4u))

/* Alias word address of LSION bit */
#define RCC_LSION_BIT_NUMBER                    (0u)
#define RCC_CSR_LSION_BB                        (PERIPH_BB_BASEADDR + (RCC_CSR_OFFSET * 32u) + (RCC_LSION_BIT_NUMBER * 4u))

/*
 * RCC HSI Configuration
 */

#define RCC_HSI_ENABLE()                        (*(volatile uint32_t *)RCC_CR_HSION_BB = ENABLE)
#define RCC_HSI_DISABLE()                       (*(volatile uint32_t *)RCC_CR_HSION_BB = DISABLE)

/*
 * RCC LSE Configuration
 */

#define RCC_LSE_ENABLE()                        (*(volatile uint32_t *)RCC_BDCR_LSEON_BB = ENABLE)
#define RCC_LSE_DISABLE()                       (*(volatile uint32_t *)RCC_BDCR_LSEON_BB = DISABLE)

/*
 * RCC LSI Configuration
 */

#define RCC_LSI_ENABLE()                        (*(volatile uint32_t *)RCC_CSR_LSION_BB = ENABLE)
#define RCC_LSI_DISABLE()                       (*(volatile uint32_t *)RCC_CSR_LSION_BB = DISABLE)

/*
 * RCC PLL Configuration
 */

#define RCC_PLL_ENABLE()                        (*(volatile uint32_t *)RCC_CR_PLLON_BB = ENABLE)
#define RCC_PLL_DISABLE()                       (*(volatile uint32_t *)RCC_CR_PLLON_BB = DISABLE)

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
    uint32_t PLLM;
    uint32_t PLLN;
    uint32_t PLLP;
    uint32_t PLLQ;
} st_RCC_PLLInitTypeDef_t;

/*
 * RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
 */

typedef struct
{
    uint32_t OscillatorType;
    uint32_t HSEState;
    uint32_t LSEState;
    uint32_t HSIState;
    uint32_t LSIState;
    uint32_t HSICalibrationValue;
    st_RCC_PLLInitTypeDef_t PLL;
} st_RCC_OscillatorInitTypeDef_t;

/*
 * RCC System, AHB and APB busses clock configuration structure definition
 */

typedef struct
{
    uint32_t ClockType;
    uint32_t SystemClockSource;
    uint32_t AHB_ClockDivider;
    uint32_t APB1_ClockDivider;
    uint32_t APB2_ClockDivider;
} st_RCC_ClockInitTypeDef_t;

e_StatusTypeDef_t RCC_OscillatorConfig(st_RCC_OscillatorInitTypeDef_t *pRCC_Oscillator);
e_StatusTypeDef_t RCC_ClockConfig(st_RCC_ClockInitTypeDef_t *pRCC_Clock);
void RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);

#endif /* INC_STM32F411XE_RCC_DRIVER_H_ */