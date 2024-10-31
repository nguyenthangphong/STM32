#include <string.h>
#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_spi_driver.h"
#include "delay.h"

void GPIO_SPI1_Init(void);
void GPIO_SPI2_Init(void);
void SPI1_Init(void);
void SPI2_Init(void);

int main(void)
{
    /* GPIO SPI1, SPI2 Init */
    GPIO_SPI1_Init();

    /* SPI1, SPI2 Init */
    SPI1_Init();

    /* Enabling SSOE to enable NSS Output */
    SPI_SSOEConfig(SPI1, ENABLE);

    /* Delay */
    delay(500000);

    uint8_t	pTxBuffer = 0x50u;

    /* Send data from SPI1 */
    SPI_SendData(SPI1, &pTxBuffer, 1);

    while (1);

    return 0;
}

void GPIO_SPI1_Init(void)
{
    st_GPIO_Handle_t GPIO_SPI1_Handle;
    memset(&GPIO_SPI1_Handle, 0, sizeof(GPIO_SPI1_Handle));

    GPIO_SPI1_Handle.pGPIOx                             = GPIOA;
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinAltFunMode  = 5;
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;

    /* NSS */
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_4;
    GPIO_Init(&GPIO_SPI1_Handle);

    /* SCLK */
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_5;
    GPIO_Init(&GPIO_SPI1_Handle);

    /* MISO */
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_6;
    GPIO_Init(&GPIO_SPI1_Handle);

    /* MOSI */
    GPIO_SPI1_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_7;
    GPIO_Init(&GPIO_SPI1_Handle);
}

void GPIO_SPI2_Init(void)
{
    st_GPIO_Handle_t GPIO_SPI2_Handle;
    memset(&GPIO_SPI2_Handle, 0, sizeof(GPIO_SPI2_Handle));

    GPIO_SPI2_Handle.pGPIOx                             = GPIOB;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinAltFunMode  = 5;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;

    /* NSS */
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
    GPIO_Init(&GPIO_SPI2_Handle);

    /* SCLK */
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_13;
    GPIO_Init(&GPIO_SPI2_Handle);

    /* MISO */
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_14;
    GPIO_Init(&GPIO_SPI2_Handle);

    /* MOSI */
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_15;
    GPIO_Init(&GPIO_SPI2_Handle);
}

void SPI1_Init(void)
{
    st_SPI_Handle_t SPI1_Handle;
    memset(&SPI1_Handle, 0, sizeof(SPI1_Handle));

    SPI1_Handle.pSPIx                     = SPI1;
    SPI1_Handle.SPI_Config.SPI_BugConfig  = SPI_BUS_CONFIG_FULLDUPLEX;
    SPI1_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1_Handle.SPI_Config.SPI_SCLKSpeed  = SPI_SCLK_SPEED_DIV_8;
    SPI1_Handle.SPI_Config.SPI_DFF        = SPI_DFF_8_BITS_DATA;
    SPI1_Handle.SPI_Config.SPI_CPOL       = SPI_CPOL_HIGH;
    SPI1_Handle.SPI_Config.SPI_CPHA       = SPI_CPHA_LOW;
    SPI1_Handle.SPI_Config.SPI_SSM        = SPI_SSM_DI;

    SPI_Init(&SPI1_Handle);
}

void SPI2_Init(void)
{
    st_SPI_Handle_t SPI2_Handle;
    memset(&SPI2_Handle, 0, sizeof(SPI2_Handle));

    SPI2_Handle.pSPIx                     = SPI2;
    SPI2_Handle.SPI_Config.SPI_BugConfig  = SPI_BUS_CONFIG_FULLDUPLEX;
    SPI2_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
    SPI2_Handle.SPI_Config.SPI_SCLKSpeed  = SPI_SCLK_SPEED_DIV_8;
    SPI2_Handle.SPI_Config.SPI_DFF        = SPI_DFF_8_BITS_DATA;
    SPI2_Handle.SPI_Config.SPI_CPOL       = SPI_CPOL_HIGH;
    SPI2_Handle.SPI_Config.SPI_CPHA       = SPI_CPHA_LOW;
    SPI2_Handle.SPI_Config.SPI_SSM        = SPI_SSM_DI;

    SPI_Init(&SPI2_Handle);
}