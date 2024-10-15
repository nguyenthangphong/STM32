#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "delay.h"
#include <string.h>

void GPIO_BTN_Init(void);
void GPIO_SPI2_Init(void);
void SPI2_Init(void);

int main(void)
{
    char data[] = "Hello World";

    /* GPIO BTN Init */
    GPIO_BTN_Init();

    /* GPIO SPI2 Init */
    GPIO_SPI2_Init();

    /* SPI2 Init */
    SPI2_Init();

    /* Enabling SSOE to enable NSS Output */
    SPI_SSOEConfig(SPI2, ENABLE);

    while (1)
    {
        /* When button is pressed, exit from this */
        while (!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

        /* Delay */
        delay(500000);

        /* Enable the SPI2 Peripheral */
        SPI_PeripheralControl(SPI2, ENABLE);

        /* Send length of data*/
        uint8_t length = strlen(data);
        SPI_SendData(SPI2, &length, 1);

        /* Send data */
        // SPI_SendData(SPI2, (uint8_t *)data, strlen(data));

        /* Checking whether the SPI2 is busy or not */
        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

        /* Disable the SPI2 Peripheral */
        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}

void GPIO_BTN_Init(void)
{
    st_GPIO_Handle_t GPIO_BTN_Handle;
    memset(&GPIO_BTN_Handle, 0, sizeof(GPIO_BTN_Handle));

    GPIO_BTN_Handle.pGPIOx = GPIOA;
    GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_BTN_Handle);
}

void GPIO_SPI2_Init(void)
{
    st_GPIO_Handle_t GPIO_SPI2_Handle;
    memset(&GPIO_SPI2_Handle, 0, sizeof(GPIO_SPI2_Handle));

    GPIO_SPI2_Handle.pGPIOx = GPIOB;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* SCLK */
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&GPIO_SPI2_Handle);

    /* MOSI */
    GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&GPIO_SPI2_Handle);

    /* MISO */
    // GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // GPIO_Init(&GPIO_SPI2_Handle);

    /* NSS */
    // GPIO_SPI2_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    // GPIO_Init(&GPIO_SPI2_Handle);
}

void SPI2_Init(void)
{
    st_SPI_Handle_t SPI2_Handle;
    memset(&SPI2_Handle, 0, sizeof(SPI2_Handle));

    SPI2_Handle.pSPIx = SPI2;
    SPI2_Handle.SPI_Config.SPI_BugConfig = SPI_BUS_CONFIG_FULLDUPLEX;
    SPI2_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2_Handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV_2;
    SPI2_Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS_DATA;
    SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    SPI2_Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI2_Handle);
}