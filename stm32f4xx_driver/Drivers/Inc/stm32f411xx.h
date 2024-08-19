#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

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
} GPIO_RegDef_t;

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
} RCC_RegDef_t;

/*
 * Peripheral Definition
 */

#define GPIOA                   ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB                   ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC                   ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD                   ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE                   ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOH                   ((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC                     ((RCC_RegDef_t *)RCC_BASEADDR)

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
 * Some Generic Macros
 */

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            ENABLE
#define GPIO_PIN_RESET          DISABLE

#endif /* INC_STM32F411XX_H_ */