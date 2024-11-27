#include <string.h>
#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_rcc_driver.h"
#include "stm32f411xe_spi_driver.h"
#include "stm32f411xe_flash_driver.h"
#include "delay.h"

void RCC_SystemClockConfigHSI(void);
void GPIOx_Init(void);
void SPIx_Init(void);

char data[] = "hello world";

int main(void)
{
    RCC_SystemClockConfigHSI();
    GPIOx_Init();
    SPIx_Init();

    /* Enable the SSI bit to NSS signal internally high and avoids MODF error */
    SPI_SSIConfig(SPI2, ENABLE);

    while (1)
    {
        /* Delay */
        delay(500000);

        /* Enable the SPI2 Peripheral */
        SPI_PeripheralControl(SPI2, ENABLE);

        /* Send data */
        SPI_SendData(SPI2, (uint8_t *)data, strlen(data));

        /* Disable the SPI2 Peripheral */
        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}

/*
 *  system_core_clock = f_PLL_clock_output = 8MHz
 *  APB1 = system_core_clock / 8           = 1MHz
 *  APB2 = system_core_clock / 4           = 2MHz
 */
void RCC_SystemClockConfigHSI(void)
{
    e_StatusTypeDef_t ret = STATUS_OK;

    st_RCC_OscillatorInitTypeDef_t rcc_oscillator_config = {0};

    rcc_oscillator_config.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    rcc_oscillator_config.HSIState            = RCC_HSI_ON;
    rcc_oscillator_config.HSICalibrationValue = RCC_HSITRIM_6;
    rcc_oscillator_config.PLL.PLLState        = RCC_PLL_ON;
    rcc_oscillator_config.PLL.PLLSource       = RCC_PLLSRC_HSI;
    rcc_oscillator_config.PLL.PLLM            = RCC_PLLM_DIV_40;
    rcc_oscillator_config.PLL.PLLN            = RCC_PLLN_MUL_160;
    rcc_oscillator_config.PLL.PLLP            = RCC_PLLP_DIV_8;

    ret = RCC_OscillatorConfig(&rcc_oscillator_config);

    st_RCC_ClockInitTypeDef_t rcc_clock_config = {0};

    rcc_clock_config.ClockType                = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    rcc_clock_config.SystemClockSource        = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clock_config.APB1_ClockDivider        = RCC_APB1_PRESCALER_8;
    rcc_clock_config.APB2_ClockDivider        = RCC_APB2_PRESCALER_4;

    ret = RCC_ClockConfig(&rcc_clock_config, FLASH_LATENCY_2);

    UNUSED(ret);
}

void GPIOx_Init(void)
{
    st_GPIO_Handle_t gpio_handle = {0};

    gpio_handle.pGPIOx                             = GPIOB;
    gpio_handle.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    gpio_handle.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_AF5_SPI2;
    gpio_handle.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpio_handle.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;

    /* SCLK */
    gpio_handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_13;
    GPIO_Init(&gpio_handle);

    /* MOSI */
    gpio_handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_15;
    GPIO_Init(&gpio_handle);
}

/*
 *  f_APB1 = 1MHz
 *  SCLK = f_APB1 / 2 = 500KHz (PB13) => T = 1 / 500KHz = 200us
 *  fs >= 2 * f_APB1 = 2 * 1MHz = 2MS/s
 */
void SPIx_Init(void)
{
    e_StatusTypeDef_t ret = STATUS_OK;

    st_SPI_Handle_t spi2_handle = {0};

    spi2_handle.pSPIx                           = SPI2;
    spi2_handle.SPI_Config.SPI_BIDIMODE_RXONLY  = SPI_BIDIMODE_RXONLY_FULLDUPLEX;
    spi2_handle.SPI_Config.SPI_MSTR             = SPI_MSTR_MASTER;
    spi2_handle.SPI_Config.SPI_BR               = SPI_BR_DIV_2;
    spi2_handle.SPI_Config.SPI_DFF              = SPI_DFF_8_BITS_DATA;
    spi2_handle.SPI_Config.SPI_CPOL             = SPI_CPOL_HIGH;
    spi2_handle.SPI_Config.SPI_CPHA             = SPI_CPHA_LOW;
    spi2_handle.SPI_Config.SPI_SSM              = SPI_SSM_EN;

    ret = SPI_Init(&spi2_handle);

    UNUSED(ret);
}