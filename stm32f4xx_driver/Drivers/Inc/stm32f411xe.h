#ifndef INC_STM32F411XE_H_
#define INC_STM32F411XE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define weak 		                            __attribute__((weak))

/*
 * ARM Cortex Mx Processor NVIC ISEx Interrupt Set-enable Register Address
 */

#define NVIC_ISER0                              ((volatile uint32_t *)0xE000E100U)
#define NVIC_ISER1                              ((volatile uint32_t *)0xE000E104U)
#define NVIC_ISER2                              ((volatile uint32_t *)0xE000E108U)
#define NVIC_ISER3                              ((volatile uint32_t *)0xE000E10CU)
#define NVIC_ISER4                              ((volatile uint32_t *)0xE000E110U)
#define NVIC_ISER5                              ((volatile uint32_t *)0xE000E114U)
#define NVIC_ISER6                              ((volatile uint32_t *)0xE000E118U)
#define NVIC_ISER7                              ((volatile uint32_t *)0xE000E11CU)

/*
 * ARM Cortex Mx Processor NVIC ICEx Interrupt Clear-enable Register Address
 */

#define NVIC_ICER0                              ((volatile uint32_t *)0XE000E180U)
#define NVIC_ICER1                              ((volatile uint32_t *)0xE000E184U)
#define NVIC_ICER2                              ((volatile uint32_t *)0xE000E188U)
#define NVIC_ICER3                              ((volatile uint32_t *)0xE000E18CU)
#define NVIC_ICER4                              ((volatile uint32_t *)0xE000E190U)
#define NVIC_ICER5                              ((volatile uint32_t *)0xE000E194U)
#define NVIC_ICER6                              ((volatile uint32_t *)0xE000E198U)
#define NVIC_ICER7                              ((volatile uint32_t *)0xE000E19CU)

/*
 * ARM Cortex Mx Processor NVIC Interrupt Priority Register Address
 */

#define NVIC_PR_BASEADDR                        ((volatile uint32_t *)0xE000E400U)
#define NO_PR_BITS_IMPLEMENTED                  4

/*
 * Base Address of Flash and SRAM memories
 */

#define FLASH_BASEADDR                          0x08000000U
#define FLASH_END_BASEADDR                      0x0807FFFFU
#define SRAM1_BASEADDR                          0x20000000U
#define SRAM1_BIT_BAND_BASEADDR                 0x22000000U
#define FLASH_OTP_BASEADDR                      0x1FFF7800U
#define FLASH_OTP_END_BASEADDR                  0x1FFF7A0FU               
#define PERIPH_BASEADDR                         0x40000000U
#define PERIPH_BB_BASEADDR                      0x42000000U

/*
 * AHBx and APBx Bus Peripheral Base Address 
 */

#define APB1PERIPH_BASEADDR                     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR                     0x40010000U
#define AHB1PERIPH_BASEADDR                     0x40020000U
#define AHB2PERIPH_BASEADDR                     0x50000000U

/*
 * Base Address of Peripherals which are hanging on APB1 Bus
 */

#define TIM2_BASEADDR                           (APB1PERIPH_BASEADDR + 0x0000U)
#define TIM3_BASEADDR                           (APB1PERIPH_BASEADDR + 0x0400U)
#define TIM4_BASEADDR                           (APB1PERIPH_BASEADDR + 0x0800U)
#define TIM5_BASEADDR                           (APB1PERIPH_BASEADDR + 0x0C00U)
#define RTC_BKP_REGISTERS_BASEADDR              (APB1PERIPH_BASEADDR + 0x2800U)
#define WWDG_BASEADDR                           (APB1PERIPH_BASEADDR + 0x2C00U)
#define IWDG_BASEADDR                           (APB1PERIPH_BASEADDR + 0x3000U)
#define I2S2EXT_BASEADDR                        (APB1PERIPH_BASEADDR + 0x3400U)
#define SPI2_BASEADDR                           (APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR                           (APB1PERIPH_BASEADDR + 0x3C00U)
#define I2S3EXT_BASEADDR                        (APB1PERIPH_BASEADDR + 0x4000U)
#define USART2_BASEADDR                         (APB1PERIPH_BASEADDR + 0x4400U)
#define I2C1_BASEADDR                           (APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR                           (APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR                           (APB1PERIPH_BASEADDR + 0x5C00U)
#define PWR_BASEADDR                            (APB1PERIPH_BASEADDR + 0x7000U)

/*
 * Base Address of Peripherals which are hanging on APB2 Bus
 */

#define TIM1_BASEADDR                           (APB2PERIPH_BASEADDR + 0x0000U)
#define USART1_BASEADDR                         (APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR                         (APB2PERIPH_BASEADDR + 0x1400U)
#define ADC1_BASEADDR                           (APB2PERIPH_BASEADDR + 0x2000U)
#define SDIO_BASEADDR                           (APB2PERIPH_BASEADDR + 0x2C00U)
#define SPI1_BASEADDR                           (APB2PERIPH_BASEADDR + 0x3000U)
#define SPI4_BASEADDR                           (APB2PERIPH_BASEADDR + 0x3400U)
#define SYSCFG_BASEADDR                         (APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR                           (APB2PERIPH_BASEADDR + 0x3C00U)
#define TIM9_BASEADDR                           (APB2PERIPH_BASEADDR + 0x4000U)
#define TIM10_BASEADDR                          (APB2PERIPH_BASEADDR + 0x4400U)
#define TIM11_BASEADDR                          (APB2PERIPH_BASEADDR + 0x4800U)
#define SPI5_BASEADDR                           (APB2PERIPH_BASEADDR + 0x5000U)

/*
 * Base Address of Peripherals which are hanging on AHB1 Bus
 */

#define GPIOA_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOH_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x1C00U)
#define CRC_BASEADDR                            (AHB1PERIPH_BASEADDR + 0x3000U)
#define RCC_BASEADDR                            (AHB1PERIPH_BASEADDR + 0x3800U)
#define FLASH_INTERFACE_REGISTER_BASEADDR       (AHB1PERIPH_BASEADDR + 0x3C00U)
#define DMA1_BASEADDR                           (AHB1PERIPH_BASEADDR + 0x6000U)
#define DMA2_BASEADDR                           (AHB1PERIPH_BASEADDR + 0x6400U)

/*
 * Base Address of Peripherals which are hanging on AHB2 Bus
 */

#define USB_OTG_FS_BASEADDR                     (AHB2PERIPH_BASEADDR + 0x0000U)

/*
 * Peripheral Register Definition Structures
 */

typedef struct
{
    volatile uint32_t            MODER;                  /*!< GPIO port mode register,                                              Address offset : 0x00 */
    volatile uint32_t            OTYPER;                 /*!< GPIO port output type register,                                       Address offset : 0x04 */
    volatile uint32_t            OSPEEDR;                /*!< GPIO port output speed register,                                      Address offset : 0x08 */
    volatile uint32_t            PUPDR;                  /*!< GPIO port pull-up/pull-down register,                                 Address offset : 0x0C */
    volatile uint32_t            IDR;                    /*!< GPIO port input data register,                                        Address offset : 0x10 */
    volatile uint32_t            ODR;                    /*!< GPIO port output data register,                                       Address offset : 0x14 */
    volatile uint32_t            BSRR;                   /*!< GPIO port bit set/reset register,                                     Address offset : 0x18 */
    volatile uint32_t            LCKR;                   /*!< GPIO port configuration lock register,                                Address offset : 0x1C */
    volatile uint32_t            AFR[2];                 /*!< GPIO alternate function registers,                                    Address offset : 0x20 - 0x24 */
} st_GPIO_RegDef_t;

typedef struct
{
    volatile uint32_t           CR;                      /*!< RCC clock control register,                                           Address offset : 0x00 */
    volatile uint32_t           PLLCFGR;                 /*!< RCC PLL configuration register,                                       Address offset : 0x04 */
    volatile uint32_t           CFGR;                    /*!< RCC clock configuration register,                                     Address offset : 0x08 */
    volatile uint32_t           CIR;                     /*!< RCC clock interrupt register,                                         Address offset : 0x0C */
    volatile uint32_t           AHB1RSTR;                /*!< RCC AHB1 peripheral reset register,                                   Address offset : 0x10 */
    volatile uint32_t           AHB2RSTR;                /*!< RCC AHB2 peripheral reset register,                                   Address offset : 0x14 */
    uint32_t                    RESERVED0[2];            /*!< Reserved,                                                             Address offset : 0x18 - 0x1C */
    volatile uint32_t           APB1RSTR;                /*!< RCC APB1 peripheral reset register,                                   Address offset : 0x20 */
    volatile uint32_t           APB2RSTR;                /*!< RCC APB2 peripheral reset register,                                   Address offset : 0x24 */
    uint32_t                    RESERVED1[2];            /*!< Reserved,                                                             Address offset : 0x28 - 0x2C */
    volatile uint32_t           AHB1ENR;                 /*!< RCC AHB1 peripheral clock enable register,                            Address offset : 0x30 */
    volatile uint32_t           AHB2ENR;                 /*!< RCC AHB2 peripheral clock enable register,                            Address offset : 0x34 */
    uint32_t                    RESERVED2[2];            /*!< Reserved,                                                             Address offset : 0x38 - 0x3C */
    volatile uint32_t           APB1ENR;                 /*!< RCC APB1 peripheral clock enable register,                            Address offset : 0x40 */
    volatile uint32_t           APB2ENR;                 /*!< RCC APB2 peripheral clock enable register,                            Address offset : 0x44 */
    uint32_t                    RESERVED3[2];            /*!< Reserved,                                                             Address offset : 0x48 - 0x4C */
    volatile uint32_t           AHB1LPENR;               /*!< RCC AHB1 peripheral clock enable in low power mode register,          Address offset : 0x50 */
    volatile uint32_t           AHB2LPENR;               /*!< RCC AHB2 peripheral clock enable in low power mode register,          Address offset : 0x54 */
    uint32_t                    RESERVED4[2];            /*!< Reserved,                                                             Address offset : 0x58 - 0x5C */
    volatile uint32_t           APB1LPENR;               /*!< RCC APB1 peripheral clock enable in low power mode register,          Address offset : 0x60 */
    volatile uint32_t           APB2LPENR;               /*!< RCC APB2 peripheral clock enable in low power mode register,          Address offset : 0x64 */
    uint32_t                    RESERVED5[2];            /*!< Reserved,                                                             Address offset : 0x68 - 0x6C */
    volatile uint32_t           BDCR;                    /*!< RCC Backup domain control register,                                   Address offset : 0x70 */
    volatile uint32_t           CSR;                     /*!< RCC clock control & status register,                                  Address offset : 0x74 */
    uint32_t                    RESERVED6[2];            /*!< Reserved,                                                             Address offset : 0x78 - 0x7C */
    volatile uint32_t           SSCGR;                   /*!< RCC spread spectrum clock generation register,                        Address offset : 0x80 */
    volatile uint32_t           PLLI2SCFGR;              /*!< RCC PLLI2S configuration register,                                    Address offset : 0x84 */
    uint32_t                    RESERVED7;               /*!< Reserved,                                                             Address offset : 0x88 */
    volatile uint32_t           DCKCFGR;                 /*!< RCC Dedicated Clocks Configuration Register,                          Address offset : 0x8C */
} st_RCC_RegDef_t;

typedef struct
{
    volatile uint32_t           IMR;                     /*!< Interrupt mask register,                                              Address offset : 0x00 */
    volatile uint32_t           EMR;                     /*!< Event mask register,                                                  Address offset : 0x04 */
    volatile uint32_t           RTSR;                    /*!< Rising trigger selection register,                                    Address offset : 0x08 */
    volatile uint32_t           FTSR;                    /*!< Falling trigger selection register,                                   Address offset : 0x0C */
    volatile uint32_t           SWIER;                   /*!< Software interrupt event register,                                    Address offset : 0x10 */
    volatile uint32_t           PR;                      /*!< Pending register,                                                     Address offset : 0x14 */
} st_EXTI_RegDef_t;

typedef struct
{
    volatile uint32_t           MEMRMP;                  /*!< SYSCFG memory remap register,                                         Address offset : 0x00 */
    volatile uint32_t           PMC;                     /*!< SYSCFG peripheral mode configuration register,                        Address offset : 0x04 */
    volatile uint32_t           EXTICR[4];               /*!< SYSCFG external interrupt configuration register,                     Address offset : 0x08 - 0x14 */
    uint32_t                    RESERVED[2];             /*!< Reserved,                                                             Address offset : 0x18 - 0x1C */
    volatile uint32_t           CMPCR;                   /*!< Compensation cell control register,                                   Address offset : 0x20 */
} st_SYSCFG_RegDef_t;

typedef struct
{
    volatile uint32_t           CR1;                     /*!< SPI control register 1,                                                Address offset : 0x00 */
    volatile uint32_t           CR2;                     /*!< SPI control register 2,                                                Address offset : 0x04 */
    volatile uint32_t           SR;                      /*!< SPI status register,                                                   Address offset : 0x08 */
    volatile uint32_t           DR;                      /*!< SPI data register,                                                     Address offset : 0x0C */
    volatile uint32_t           CRCPR;                   /*!< SPI CRC polynomial register,                                           Address offset : 0x10 */
    volatile uint32_t           RXCRCR;                  /*!< SPI RX CRC register,                                                   Address offset : 0x14 */
    volatile uint32_t           TXCRCR;                  /*!< SPI TX CRC register,                                                   Address offset : 0x18 */
    volatile uint32_t           I2SCFGR;                 /*!< SPI I2S configuration register,                                        Address offset : 0x1C */
    volatile uint32_t           I2SPR;                   /*!< SPI I2S prescaler register,                                            Address offset : 0x20 */
} st_SPI_RegDef_t;

typedef struct
{
    volatile uint32_t           SR;                      /*!< USART Status register,                                                 Address offset : 0x00 */
    volatile uint32_t           DR;                      /*!< USART Data register,                                                   Address offset : 0x04 */
    volatile uint32_t           BRR;                     /*!< USART Baud rate register,                                              Address offset : 0x08 */
    volatile uint32_t           CR1;                     /*!< USART Control register 1,                                              Address offset : 0x0C */
    volatile uint32_t           CR2;                     /*!< USART Control register 2,                                              Address offset : 0x10 */
    volatile uint32_t           CR3;                     /*!< USART Control register 3,                                              Address offset : 0x14 */
    volatile uint32_t           GTPR;                    /*!< USART Guard time and prescaler register,                               Address offset : 0x18 */
} st_USART_RegDef_t;

/*
 * SPI Pin configuration of CR1 Register
 */

#define SPI_CR1_CPHA            0
#define SPI_CR1_CPOL            1
#define SPI_CR1_MSTR            2
#define SPI_CR1_BR              3
#define SPI_CR1_SPE             6
#define SPI_CR1_LSBFIRST        7
#define SPI_CR1_SSI             8
#define SPI_CR1_SSM             9
#define SPI_CR1_RXONLY          10
#define SPI_CR1_DFF             11
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_CRCEN           13
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_BIDIMODE        15

/*
 * SPI Pin configuration of CR2 Register
 */

#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_SSOE            2
#define SPI_CR2_FRF             4
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXEIE           7

/*
 * SPI Pin configuration of SR Register
 */

#define SPI_SR_RXNE             0
#define SPI_SR_TXE              1
#define SPI_SR_CHSIDE           2
#define SPI_SR_UDR              3
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7
#define SPI_SR_FRE              8

/*
 * USART Pin configuration of SR Register
 */

#define USART_SR_PE                     0
#define USART_SR_FE                     1
#define USART_SR_NF                     2
#define USART_SR_ORE                    3
#define USART_SR_IDLE                   4
#define USART_SR_RXNE                   5
#define USART_SR_TC                     6
#define USART_SR_TXE                    7
#define USART_SR_LBD                    8
#define USART_SR_CTS                    9

/*
 * USART Pin configuration of BRR Register
 */

#define USART_BRR_DIV_FRACTION          0
#define USART_BRR_DIV_MANTISSA          4

/*
 * USART Pin configuration of CR1 Register
 */

#define USART_CR1_SBK                   0
#define USART_CR1_RWU                   1
#define USART_CR1_RE                    2
#define USART_CR1_TE                    3
#define USART_CR1_IDLEIE                4
#define USART_CR1_RXNEIE                5
#define USART_CR1_TCIE                  6
#define USART_CR1_TXEIE                 7
#define USART_CR1_PEIE                  8
#define USART_CR1_PS                    9
#define USART_CR1_PCE                   10
#define USART_CR1_WAKE                  11
#define USART_CR1_M                     12
#define USART_CR1_UE                    13
#define USART_CR1_OVER8                 15

/*
 * USART Pin configuration of CR2 Register
 */

#define USART_CR2_ADD                   0
#define USART_CR2_LBDL                  5
#define USART_CR2_LBDIE                 6
#define USART_CR2_LBCL                  8
#define USART_CR2_CPHA                  9
#define USART_CR2_CPOL                  10
#define USART_CR2_CLKEN                 11
#define USART_CR2_STOP                  12
#define USART_CR2_LINEN                 14

/*
 * USART Pin configuration of CR3 Register
 */

#define USART_CR3_EIE                   0
#define USART_CR3_IREN                  1
#define USART_CR3_IRLP                  2
#define USART_CR3_HDSEL                 3
#define USART_CR3_NACK                  4
#define USART_CR3_SCEN                  5
#define USART_CR3_DMAR                  6
#define USART_CR3_DMAT                  7
#define USART_CR3_RTSE                  8
#define USART_CR3_CTSE                  9
#define USART_CR3_CTSIE                 10
#define USART_CR3_ONEBIT                11

/***************************
 * Reset and Clock Control *
 **************************/

/**************************************** Bit definition for RCC_CR register ****************************************/
#define RCC_CR_HSION_POSITION           (0U)
#define RCC_CR_HSION_MASK               (0x1U << RCC_CR_HSION_POSITION)                     /*!< 0x00000001 */
#define RCC_CR_HSION                    RCC_CR_HSION_MASK

#define RCC_CR_HSIRDY_POSITION          (1U)
#define RCC_CR_HSIRDY_MASK              (0x1U << RCC_CR_HSIRDY_POSITION)                    /*!< 0x00000002 */
#define RCC_CR_HSIRDY                   RCC_CR_HSIRDY_MASK

#define RCC_CR_HSITRIM_POSITION         (3U)
#define RCC_CR_HSITRIM_MASK             (0x1FU << RCC_CR_HSITRIM_POSITION)                  /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                  RCC_CR_HSITRIM_MASK
#define RCC_CR_HSITRIM_0                (0x01U << RCC_CR_HSITRIM_POSITION)                  /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                (0x02U << RCC_CR_HSITRIM_POSITION)                  /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                (0x04U << RCC_CR_HSITRIM_POSITION)                  /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                (0x08U << RCC_CR_HSITRIM_POSITION)                  /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                (0x10U << RCC_CR_HSITRIM_POSITION)                  /*!< 0x00000080 */

#define RCC_CR_HSICAL_POSITION          (8U)
#define RCC_CR_HSICAL_MASK              (0xFFU << RCC_CR_HSICAL_POSITION)                   /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                   RCC_CR_HSICAL_MASK

#define RCC_CR_HSEON_POSITION           (16U)
#define RCC_CR_HSEON_MASK               (0x1U << RCC_CR_HSEON_POSITION)                     /*!< 0x00010000 */
#define RCC_CR_HSEON                    RCC_CR_HSEON_MASK

#define RCC_CR_HSERDY_POSITION          (17U)
#define RCC_CR_HSERDY_MASK              (0x1U << RCC_CR_HSERDY_POSITION)                    /*!< 0x00020000 */
#define RCC_CR_HSERDY                   RCC_CR_HSERDY_MASK

#define RCC_CR_HSEBYP_POSITION          (18U)
#define RCC_CR_HSEBYP_MASK              (0x1U << RCC_CR_HSEBYP_POSITION)                    /*!< 0x00040000 */
#define RCC_CR_HSEBYP                   RCC_CR_HSEBYP_MASK

#define RCC_CR_CSSON_POSITION           (19U)
#define RCC_CR_CSSON_MASK               (0x1U << RCC_CR_CSSON_POSITION)                     /*!< 0x00080000 */
#define RCC_CR_CSSON                    RCC_CR_CSSON_MASK

#define RCC_CR_PLLON_POSITION           (24U)
#define RCC_CR_PLLON_MASK               (0x1U << RCC_CR_PLLON_POSITION)                     /*!< 0x01000000 */
#define RCC_CR_PLLON                    RCC_CR_PLLON_MASK

#define RCC_CR_PLLRDY_POSITION          (25U)
#define RCC_CR_PLLRDY_MASK              (0x1U << RCC_CR_PLLRDY_POSITION)                    /*!< 0x02000000 */

#define RCC_CR_PLLI2SON_POSITION        (26U)
#define RCC_CR_PLLI2SON_MASK            (0x1U << RCC_CR_PLLI2SON_POSITION)                  /*!< 0x04000000 */
#define RCC_CR_PLLI2SON                 RCC_CR_PLLI2SON_MASK

#define RCC_CR_PLLI2SRDY_POSITION       (27U)
#define RCC_CR_PLLI2SRDY_MASK           (0x1U << RCC_CR_PLLI2SRDY_POSITION)                 /*!< 0x08000000 */
#define RCC_CR_PLLI2SRDY                RCC_CR_PLLI2SRDY_MASK

/**************************************** Bit definition for RCC_PLLCFGR register ****************************************/
#define RCC_PLLCFGR_PLLM_POSITION       (0U)
#define RCC_PLLCFGR_PLLM_MASK           (0x3FU << RCC_PLLCFGR_PLLM_POSITION)                /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM                RCC_PLLCFGR_PLLM_MASK
#define RCC_PLLCFGR_PLLM_0              (0x01U << RCC_PLLCFGR_PLLM_POSITION)                /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLM_1              (0x02U << RCC_PLLCFGR_PLLM_POSITION)                /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLM_2              (0x04U << RCC_PLLCFGR_PLLM_POSITION)                /*!< 0x00000004 */
#define RCC_PLLCFGR_PLLM_3              (0x08U << RCC_PLLCFGR_PLLM_POSITION)                /*!< 0x00000008 */
#define RCC_PLLCFGR_PLLM_4              (0x10U << RCC_PLLCFGR_PLLM_POSITION)                /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_5              (0x20U << RCC_PLLCFGR_PLLM_POSITION)                /*!< 0x00000020 */

#define RCC_PLLCFGR_PLLN_POSITION       (6U)
#define RCC_PLLCFGR_PLLN_MASK           (0x1FFU << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN                RCC_PLLCFGR_PLLN_MASK
#define RCC_PLLCFGR_PLLN_0              (0x001U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00000040 */
#define RCC_PLLCFGR_PLLN_1              (0x002U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00000080 */
#define RCC_PLLCFGR_PLLN_2              (0x004U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_3              (0x008U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_4              (0x010U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_5              (0x020U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_6              (0x040U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_7              (0x080U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_8              (0x100U << RCC_PLLCFGR_PLLN_POSITION)               /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLP_POSITION       (16U)
#define RCC_PLLCFGR_PLLP_MASK           (0x3U << RCC_PLLCFGR_PLLP_POSITION)                 /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP                RCC_PLLCFGR_PLLP_MASK
#define RCC_PLLCFGR_PLLP_0              (0x1U << RCC_PLLCFGR_PLLP_POSITION)                 /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLP_1              (0x2U << RCC_PLLCFGR_PLLP_POSITION)                 /*!< 0x00020000 */

#define RCC_PLLCFGR_PLLSRC_POSITION     (22U)
#define RCC_PLLCFGR_PLLSRC_MASK         (0x1U << RCC_PLLCFGR_PLLSRC_POSITION)               /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC              RCC_PLLCFGR_PLLSRC_MASK

#define RCC_PLLCFGR_PLLQ_POSITION       (24U)
#define RCC_PLLCFGR_PLLQ_MASK           (0xFU << RCC_PLLCFGR_PLLSRC_POSITION)               /*!< 0x0F000000 */
#define RCC_PLLCFGR_PLLQ                RCC_PLLCFGR_PLLQ_MASK
#define RCC_PLLCFGR_PLLQ_0              (0x1U << RCC_PLLCFGR_PLLSRC_POSITION)               /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQ_1              (0x2U << RCC_PLLCFGR_PLLSRC_POSITION)               /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_2              (0x4U << RCC_PLLCFGR_PLLSRC_POSITION)               /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_3              (0x8U << RCC_PLLCFGR_PLLSRC_POSITION)               /*!< 0x08000000 */

/**************************************** Bit definition for RCC_CFGR register ****************************************/
#define RCC_CFGR_SW_POSITION            (0U)
#define RCC_CFGR_SW_MASK                (0x3U << RCC_CFGR_SW_POSITION)                      /*!< 0x00000003 */
#define RCC_CFGR_SW                     RCC_CFGR_SW_MASK
#define RCC_CFGR_SW_0                   (0x1U << RCC_CFGR_SW_POSITION)                      /*!< 0x00000001 */
#define RCC_CFGR_SW_1                   (0x2U << RCC_CFGR_SW_POSITION)                      /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                 (0x00000000U)                                       /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                 (0x00000001U)                                       /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                 (0x00000002U)                                       /*!< PLL selected as system clock */

#define RCC_CFGR_SWS_POSITION           (2U)
#define RCC_CFGR_SWS_MASK               (0x3U << RCC_CFGR_SWS_POSITION)                     /*!< 0x0000000C */
#define RCC_CFGR_SWS                    RCC_CFGR_SWS_MASK
#define RCC_CFGR_SWS_0                  (0x1U << RCC_CFGR_SWS_POSITION)                     /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                  (0x2U << RCC_CFGR_SWS_POSITION)                     /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                (0x00000000U)                                       /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                (0x00000004U)                                       /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                (0x00000008U)                                       /*!< PLL oscillator used as system clock */

#define RCC_CFGR_HPRE_POSITION          (4U)
#define RCC_CFGR_HPRE_MASK              (0xFU << RCC_CFGR_HPRE_POSITION)                    /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                   RCC_CFGR_HPRE_MASK
#define RCC_CFGR_HPRE_0                 (0x1U << RCC_CFGR_HPRE_POSITION)                    /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                 (0x2U << RCC_CFGR_HPRE_POSITION)                    /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                 (0x4U << RCC_CFGR_HPRE_POSITION)                    /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                 (0x8U << RCC_CFGR_HPRE_POSITION)                    /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV_1             (0x00000000U)                                       /*!< system clock not divided */
#define RCC_CFGR_HPRE_DIV_2             (0x00000080U)                                       /*!< system clock divided by 2 */
#define RCC_CFGR_HPRE_DIV_4             (0x00000090U)                                       /*!< system clock divided by 4 */
#define RCC_CFGR_HPRE_DIV_8             (0x000000A0U)                                       /*!< system clock divided by 8 */
#define RCC_CFGR_HPRE_DIV_16            (0x000000B0U)                                       /*!< system clock divided by 16 */
#define RCC_CFGR_HPRE_DIV_64            (0x000000C0U)                                       /*!< system clock divided by 64 */
#define RCC_CFGR_HPRE_DIV_128           (0x000000D0U)                                       /*!< system clock divided by 128 */
#define RCC_CFGR_HPRE_DIV_256           (0x000000E0U)                                       /*!< system clock divided by 256 */
#define RCC_CFGR_HPRE_DIV_512           (0x000000F0U)                                       /*!< system clock divided by 512 */

#define RCC_CFGR_PPRE1_POSITION         (10U)
#define RCC_CFGR_PPRE1_MASK             (0x7U << RCC_CFGR_PPRE1_POSITION)                   /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                  RCC_CFGR_PPRE1_MASK
#define RCC_CFGR_PPRE1_0                (0x1U << RCC_CFGR_PPRE1_POSITION)                   /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1                (0x2U << RCC_CFGR_PPRE1_POSITION)                   /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2                (0x4U << RCC_CFGR_PPRE1_POSITION)                   /*!< 0x00001000 */

#define RCC_CFGR_PPRE1_DIV_1            (0x00000000U)                                       /*!< AHB clock not divided */
#define RCC_CFGR_PPRE1_DIV_2            (0x00001000U)                                       /*!< AHB clock divided by 2 */
#define RCC_CFGR_PPRE1_DIV_4            (0x00001400U)                                       /*!< AHB clock divided by 4 */
#define RCC_CFGR_PPRE1_DIV_8            (0x00001800U)                                       /*!< AHB clock divided by 8 */
#define RCC_CFGR_PPRE1_DIV_16           (0x00001C00U)                                       /*!< AHB clock divided by 16 */

#define RCC_CFGR_PPRE2_POSITION         (13U)
#define RCC_CFGR_PPRE2_MASK             (0x7U << RCC_CFGR_PPRE2_POSITION)                   /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                  RCC_CFGR_PPRE2_MASK
#define RCC_CFGR_PPRE2_0                (0x1U << RCC_CFGR_PPRE2_POSITION)                   /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1                (0x2U << RCC_CFGR_PPRE2_POSITION)                   /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2                (0x4U << RCC_CFGR_PPRE2_POSITION)                   /*!< 0x00008000 */

#define RCC_CFGR_PPRE2_DIV_1            (0x00000000U)                                       /*!< AHB clock not divided */
#define RCC_CFGR_PPRE2_DIV_2            (0x00008000U)                                       /*!< AHB clock divided by 2 */
#define RCC_CFGR_PPRE2_DIV_4            (0x0000A000U)                                       /*!< AHB clock divided by 4 */
#define RCC_CFGR_PPRE2_DIV_8            (0x0000C000U)                                       /*!< AHB clock divided by 8 */
#define RCC_CFGR_PPRE2_DIV_16           (0x0000E000U)                                       /*!< AHB clock divided by 16 */

#define RCC_CFGR_RTCPRE_POSITION        (16U)
#define RCC_CFGR_RTCPRE_MASK            (0x1FU << RCC_CFGR_RTCPRE_POSITION)                 /*!< 0x001F0000 */
#define RCC_CFGR_RTCPRE                 RCC_CFGR_RTCPRE_MASK
#define RCC_CFGR_RTCPRE_0               (0x01U << RCC_CFGR_RTCPRE_POSITION)                 /*!< 0x00010000 */
#define RCC_CFGR_RTCPRE_1               (0x02U << RCC_CFGR_RTCPRE_POSITION)                 /*!< 0x00020000 */
#define RCC_CFGR_RTCPRE_2               (0x04U << RCC_CFGR_RTCPRE_POSITION)                 /*!< 0x00040000 */
#define RCC_CFGR_RTCPRE_3               (0x08U << RCC_CFGR_RTCPRE_POSITION)                 /*!< 0x00080000 */
#define RCC_CFGR_RTCPRE_4               (0x10U << RCC_CFGR_RTCPRE_POSITION)                 /*!< 0x00100000 */

#define RCC_CFGR_MCO1_POSITION          (21U)
#define RCC_CFGR_MCO1_MASK              (0x3U << RCC_CFGR_MCO1_POSITION)                    /*!< 0x00600000 */
#define RCC_CFGR_MCO1                   RCC_CFGR_MCO1_MASK
#define RCC_CFGR_MCO1_0                 (0x1U << RCC_CFGR_MCO1_POSITION)                    /*!< 0x00200000 */
#define RCC_CFGR_MCO1_1                 (0x2U << RCC_CFGR_MCO1_POSITION)                    /*!< 0x00400000 */

#define RCC_CFGR_I2SSRC_POSITION        (23U)
#define RCC_CFGR_I2SSRC_MASK            (0x1U << RCC_CFGR_I2SSRC_POSITION)                  /*!< 0x00800000 */
#define RCC_CFGR_I2SSRC                 RCC_CFGR_I2SSRC_MASK

#define RCC_CFGR_MCO1PRE_POSITION       (24U)
#define RCC_CFGR_MCO1PRE_MASK           (0x7U << RCC_CFGR_MCO1PRE_POSITION)                 /*!< 0x07000000 */
#define RCC_CFGR_MCO1PRE                RCC_CFGR_MCO1PRE_MASK
#define RCC_CFGR_MCO1PRE_0              (0x1U << RCC_CFGR_MCO1PRE_POSITION)                 /*!< 0x01000000 */
#define RCC_CFGR_MCO1PRE_1              (0x2U << RCC_CFGR_MCO1PRE_POSITION)                 /*!< 0x02000000 */
#define RCC_CFGR_MCO1PRE_2              (0x4U << RCC_CFGR_MCO1PRE_POSITION)                 /*!< 0x04000000 */

#define RCC_CFGR_MCO2PRE_POSITION       (27U)
#define RCC_CFGR_MCO2PRE_MASK           (0x7U << RCC_CFGR_MCO2PRE_POSITION)                 /*!< 0x38000000 */
#define RCC_CFGR_MCO2PRE                RCC_CFGR_MCO2PRE_MASK
#define RCC_CFGR_MCO2PRE_0              (0x1U << RCC_CFGR_MCO2PRE_POSITION)                 /*!< 0x08000000 */
#define RCC_CFGR_MCO2PRE_1              (0x2U << RCC_CFGR_MCO2PRE_POSITION)                 /*!< 0x10000000 */
#define RCC_CFGR_MCO2PRE_2              (0x4U << RCC_CFGR_MCO2PRE_POSITION)                 /*!< 0x20000000 */

#define RCC_CFGR_MCO2_POSITION          (30U)
#define RCC_CFGR_MCO2_MASK              (0x3U << RCC_CFGR_MCO2_POSITION)                    /*!< 0xC0000000 */
#define RCC_CFGR_MCO2                   RCC_CFGR_MCO2_MASK
#define RCC_CFGR_MCO2_0                 (0x1U << RCC_CFGR_MCO2_POSITION)                    /*!< 0x40000000 */
#define RCC_CFGR_MCO2_1                 (0x2U << RCC_CFGR_MCO2_POSITION)                    /*!< 0x80000000 */

/**************************************** Bit definition for RCC_CIR register ****************************************/
#define RCC_CIR_LSIRDYF                 0
#define RCC_CIR_LSERDYF                 1
#define RCC_CIR_HSIRDYF                 2
#define RCC_CIR_HSERDYF                 3
#define RCC_CIR_PLLRDYF                 4
#define RCC_CIR_PLLI2SRDYF              5
#define RCC_CIR_CSSF                    7
#define RCC_CIR_LSIRDYE                 8
#define RCC_CIR_LSERDYE                 9
#define RCC_CIR_HSIRDYE                 10
#define RCC_CIR_HSERDYE                 11
#define RCC_CIR_PLLRDYE                 12
#define RCC_CIR_PLLI2SRDYE              13
#define RCC_CIR_LSIRDYC                 16
#define RCC_CIR_LSERDYC                 17
#define RCC_CIR_HSIRDYC                 18
#define RCC_CIR_HSERDYC                 19
#define RCC_CIR_PLLRDYC                 20
#define RCC_CIR_PLLI2SRDYC              21
#define RCC_CIR_CSSC                    23

/**************************************** Bit definition for RCC_AHB1RSTR register ****************************************/
#define RCC_AHB1RSTR_GPIOARST           0
#define RCC_AHB1RSTR_GPIOBRST           1
#define RCC_AHB1RSTR_GPIOCRST           2
#define RCC_AHB1RSTR_GPIODRST           3
#define RCC_AHB1RSTR_GPIOERST           4
#define RCC_AHB1RSTR_GPIOHRST           7
#define RCC_AHB1RSTR_CRCRST             12
#define RCC_AHB1RSTR_DMA1RST            21
#define RCC_AHB1RSTR_DMA2RST            22

/**************************************** Bit definition for RCC_AHB2RSTR register ****************************************/
#define RCC_AHB2RSTR_OTGFSRST           7

/**************************************** Bit definition for RCC_APB1RSTR register ****************************************/
#define RCC_APB1RSTR_TIM2RST            0
#define RCC_APB1RSTR_TIM3RST            1
#define RCC_APB1RSTR_TIM4RST            2
#define RCC_APB1RSTR_TIM5RST            3
#define RCC_APB1RSTR_WWDGRST            11
#define RCC_APB1RSTR_SPI2RST            14
#define RCC_APB1RSTR_SPI3RST            15
#define RCC_APB1RSTR_USART2RST          17
#define RCC_APB1RSTR_I2C1RST            21
#define RCC_APB1RSTR_I2C2RST            22
#define RCC_APB1RSTR_I2C3RST            23
#define RCC_APB1RSTR_PWRRST             28

/**************************************** Bit definition for RCC_APB2RSTR register ****************************************/
#define RCC_APB2RSTR_TIM1RST            0
#define RCC_APB2RSTR_USART1RST          4
#define RCC_APB2RSTR_USART6RST          5
#define RCC_APB2RSTR_ADC1RST            8
#define RCC_APB2RSTR_SDIORST            11
#define RCC_APB2RSTR_SPI1RST            12
#define RCC_APB2RSTR_SPI4RST            13
#define RCC_APB2RSTR_SYSCFGRST          14
#define RCC_APB2RSTR_TIM9RST            16
#define RCC_APB2RSTR_TIM10RST           17
#define RCC_APB2RSTR_TIM11RST           18
#define RCC_APB2RSTR_SPI5RST            20

/**************************************** Bit definition for RCC_AHB1ENR register ****************************************/
#define RCC_AHB1ENR_GPIOAEN             0
#define RCC_AHB1ENR_GPIOBEN             1
#define RCC_AHB1ENR_GPIOCEN             2
#define RCC_AHB1ENR_GPIODEN             3
#define RCC_AHB1ENR_GPIOEEN             4
#define RCC_AHB1ENR_GPIOHEN             7
#define RCC_AHB1ENR_CRCEN               12
#define RCC_AHB1ENR_DMA1EN              21
#define RCC_AHB1ENR_DMA2EN              22

/**************************************** Bit definition for RCC_AHB2ENR register ****************************************/
#define RCC_AHB2ENR_OTGFSEN             7

/**************************************** Bit definition for RCC_APB1ENR register ****************************************/
#define RCC_APB1ENR_TIM2EN              0
#define RCC_APB1ENR_TIM3EN              1
#define RCC_APB1ENR_TIM4EN              2
#define RCC_APB1ENR_TIM5EN              3
#define RCC_APB1ENR_WWDGEN              11
#define RCC_APB1ENR_SPI2EN              14
#define RCC_APB1ENR_SPI3EN              15
#define RCC_APB1ENR_USART2EN            17
#define RCC_APB1ENR_I2C1EN              21
#define RCC_APB1ENR_I2C2EN              22
#define RCC_APB1ENR_I2C3EN              23
#define RCC_APB1ENR_PWREN               28

/**************************************** Bit definition for RCC_APB2ENR register ****************************************/
#define RCC_APB2ENR_TIM1EN              0
#define RCC_APB2ENR_USART1EN            4
#define RCC_APB2ENR_USART6EN            5
#define RCC_APB2ENR_ADC1EN              8
#define RCC_APB2ENR_SDIOEN              11
#define RCC_APB2ENR_SPI1EN              12
#define RCC_APB2ENR_SPI4EN              13
#define RCC_APB2ENR_SYSCFGEN            14
#define RCC_APB2ENR_TIM9EN              16
#define RCC_APB2ENR_TIM10EN             17
#define RCC_APB2ENR_TIM11EN             18
#define RCC_APB2ENR_SPI5EN              20

/**************************************** Bit definition for RCC_BDCR register ****************************************/
#define RCC_BDCR_LSEON                  0
#define RCC_BDCR_LSERDY                 1
#define RCC_BDCR_LSEBYP                 2
#define RCC_BDCR_LSEMOD                 3
#define RCC_BDCR_RTCSEL                 8
#define RCC_BDCR_RTCEN                  15
#define RCC_BDCR_BDRST                  16

/**************************************** Bit definition for RCC_CSR register ****************************************/
#define RCC_CSR_LSION                   0
#define RCC_CSR_LSIRDY                  1
#define RCC_CSR_RMVF                    24
#define RCC_CSR_BORRSTF                 25
#define RCC_CSR_PINRSTF                 26
#define RCC_CSR_PORRSTF                 27
#define RCC_CSR_SFTRSTF                 28
#define RCC_CSR_IWDGRSTF                29
#define RCC_CSR_WWDGRSTF                30
#define RCC_CSR_LPWRRSTF                31

/**************************************** Bit definition for RCC_SSCGR register ****************************************/
#define RCC_SSCGR_MODPER                0
#define RCC_SSCGR_INCSTEP               13
#define RCC_SSCGR_SPREADSEL             30
#define RCC_SSCGR_SSCGEN                31

/**************************************** Bit definition for RCC_PLLI2SCFGR register ****************************************/
#define RCC_PLLI2SCFGR_PLLI2SM          0
#define RCC_PLLI2SCFGR_PLLI2SN          6
#define RCC_PLLI2SCFGR_PLLI2SR          28

/**************************************** Bit definition for RCC_DCKCFGR register ****************************************/
#define RCC_DCKCFGR_TIMPRE              24

/*
 * Peripheral Definition
 */

#define GPIOA                   ((st_GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB                   ((st_GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC                   ((st_GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD                   ((st_GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE                   ((st_GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOH                   ((st_GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC                     ((st_RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI                    ((st_EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG                  ((st_SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1                    ((st_SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2                    ((st_SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3                    ((st_SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4                    ((st_SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5                    ((st_SPI_RegDef_t *)SPI5_BASEADDR)

#define USART1                  ((st_USART_RegDef_t *)USART1_BASEADDR)
#define USART2                  ((st_USART_RegDef_t *)USART2_BASEADDR)
#define USART6                  ((st_USART_RegDef_t *)USART6_BASEADDR)

/******************************* Enable **********************************/

/*
 * Clock Enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOCEN))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIODEN))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOEEN))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOHEN))

/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << RCC_APB1ENR_I2C1EN))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << RCC_APB1ENR_I2C2EN))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1 << RCC_APB1ENR_I2C3EN))

/*
 * Clock Enable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI1EN))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI2EN))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI3EN))
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI4EN))
#define SPI5_PCLK_EN()          (RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI5EN))

/*
 * Clock Enable Macros for USARTx Peripherals
 */

#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN))
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1 << RCC_APB2ENR_USART6EN))

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << RCC_APB2ENR_SYSCFGEN))

/******************************* Disable **********************************/

/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOAEN))
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOBEN))
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOCEN))
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIODEN))
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOEEN))
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOHEN))

/*
 * Clock Disable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C1EN))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C2EN))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C3EN))

/*
 * Clock Disable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI1EN))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI2EN))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI3EN))
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI4EN))
#define SPI5_PCLK_DI()          (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI5EN))

/*
 * Clock Disable Macros for USARTx Peripherals
 */

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART1EN))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << RCC_APB1ENR_USART2EN))
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART6EN))

/*
 * Clock Disable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SYSCFGEN))

/*
 * Macros to Reset GPIOx Peripherals
 */

#define GPIOA_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOARST)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOARST)); } while (0)
#define GPIOB_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOBRST)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOBRST)); } while (0)
#define GPIOC_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOCRST)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOCRST)); } while (0)
#define GPIOD_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIODRST)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIODRST)); } while (0)
#define GPIOE_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOERST)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOERST)); } while (0)
#define GPIOH_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOHRST)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOHRST)); } while (0)

/*
 * Macros to Reset SPIx Peripherals
 */

#define SPI1_REG_RESET()        do { (RCC->APB2RSTR |= (1 << RCC_APB2RSTR_SPI1RST)); (RCC->APB2RSTR &= ~(1 << RCC_APB2RSTR_SPI1RST)); } while (0)
#define SPI2_REG_RESET()        do { (RCC->APB1RSTR |= (1 << RCC_APB1RSTR_SPI2RST)); (RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_SPI2RST)); } while (0)
#define SPI3_REG_RESET()        do { (RCC->APB1RSTR |= (1 << RCC_APB1RSTR_SPI3RST)); (RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_SPI3RST)); } while (0)
#define SPI4_REG_RESET()        do { (RCC->APB2RSTR |= (1 << RCC_APB2RSTR_SPI4RST)); (RCC->APB2RSTR &= ~(1 << RCC_APB2RSTR_SPI4RST)); } while (0)
#define SPI5_REG_RESET()        do { (RCC->APB2RSTR |= (1 << RCC_APB2RSTR_SPI5RST)); (RCC->APB2RSTR &= ~(1 << RCC_APB2RSTR_SPI5RST)); } while (0)

/*
 * Return port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)    ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 : (x == GPIOC) ? 2 : (x == GPIOD) ? 3 : (x == GPIOE) ? 4 : (x == GPIOH) ? 7 : 0)

/*
 * Some Generic Macros
 */

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            ENABLE
#define GPIO_PIN_RESET          DISABLE
#define FLAG_SET                SET
#define FLAG_RESET              RESET
#define UNUSED(x)               ((void)x)

/* 
 * IRQ (Interrupt Request) Numbers of STM32F411x MCU 
 */

#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40

/* 
 * Macros for all the possible priority levels
 */

#define IRQ_NO_PRIORITY_0       0
#define IRQ_NO_PRIORITY_15      15

#endif /* INC_STM32F411XE_H_ */