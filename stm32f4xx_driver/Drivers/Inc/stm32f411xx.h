#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

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

/*
 * RCC Pin configuration of CFGR Register
 */

#define RCC_CFGR_SW                     0
#define RCC_CFGR_SWS                    2
#define RCC_CFGR_HPRE                   4
#define RCC_CFGR_PPRE1                  10
#define RCC_CFGR_PPRE2                  13
#define RCC_CFGR_RTCPRE                 16
#define RCC_CFGR_MCO1                   21
#define RCC_CFGR_I2SSRC                 23
#define RCC_CFGR_MCO1PRE                24
#define RCC_CFGR_MCO2PRE                27
#define RCC_CFGR_MCO2                   30

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

#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1 << 7))

/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()          (RCC->APB2ENR |= (1 << 20))

/*
 * Clock Enable Macros for USARTx Peripherals
 */

#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))

/******************************* Disable **********************************/

/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock Disable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 20))

/*
 * Clock Disable Macros for USARTx Peripherals
 */

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to Reset GPIOx Peripherals
 */

#define GPIOA_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while (0)
#define GPIOB_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 1)); } while (0)
#define GPIOC_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 2)); } while (0)
#define GPIOD_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 3)); } while (0)
#define GPIOE_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 4)); } while (0)
#define GPIOH_REG_RESET()       do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 7)); } while (0)

/*
 * Macros to Reset SPIx Peripherals
 */

#define SPI1_REG_RESET()        do { (RCC->APB2ENR |= (1 << 12)); (RCC->APB2ENR &= ~(1 << 12)); } while (0)
#define SPI2_REG_RESET()        do { (RCC->APB1ENR |= (1 << 14)); (RCC->APB1ENR &= ~(1 << 14)); } while (0)
#define SPI3_REG_RESET()        do { (RCC->APB1ENR |= (1 << 15)); (RCC->APB1ENR &= ~(1 << 15)); } while (0)
#define SPI4_REG_RESET()        do { (RCC->APB2ENR |= (1 << 13)); (RCC->APB2ENR &= ~(1 << 13)); } while (0)
#define SPI5_REG_RESET()        do { (RCC->APB2ENR |= (1 << 20)); (RCC->APB2ENR &= ~(1 << 20)); } while (0)

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

#endif /* INC_STM32F411XX_H_ */