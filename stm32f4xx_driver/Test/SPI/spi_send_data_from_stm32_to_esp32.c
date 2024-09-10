#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "delay.h"
#include <string.h>

/*
 *  STM32 - MASTER
 *  PB15 => SPI2_MOSI => Wire Green
 *  PB14 => SPI2_MISO => Wire Red
 *  PB13 => SPI2_SCLK => Wire Gray
 *  PB12 => SPI2_NSS  => Wire Yellow
 */

/*
 *  ESP32 - SLAVE
 *  D23 => MOSI => Wire Green
 *  D19 => MISO => Wire Red
 *  D18 => SCLK => Wire Gray
 *  D5  => NSS  => Wire Yellow
 */

const char data[] = "Nguyen Thang Phong - 2401001";

int main(void)
{
    uint8_t length = strlen(data);

    /* GPIO Button Init */
    st_GPIO_Handle_t GPIO_Button_Handle;
    GPIO_Button_Handle.pGPIOx = GPIOA;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_Button_Handle);

    /* SPI2 GPIO Init */
    st_GPIO_Handle_t SPI_GPIO_Handle;
    SPI_GPIO_Handle.pGPIOx = GPIOB;
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPI_GPIO_Handle);
    // MOSI
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPI_GPIO_Handle);
    // MISO
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPI_GPIO_Handle);
    // NSS
    SPI_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPI_GPIO_Handle);

    /* SPI2 Init */
    st_SPI_Handle_t SPI_Handle;
    SPI_Handle.pSPIx = SPI2;                                            /* SPI2 */
    SPI_Handle.SPI_Config.SPI_BugConfig = SPI_BUS_CONFIG_FULLDUPLEX;    /* Full Duplex */
    SPI_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;      /* Master */
    SPI_Handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV_8;         /* Generate SCLK of 2MHz */
    SPI_Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS_DATA;                /* 8 bits */
    SPI_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    SPI_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    SPI_Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;                         /* Hardware Slave Management Enabled for NSS Pin */

    SPI_Init(&SPI_Handle);

    SPI_SSOEConfig(SPI2, ENABLE);

    while (1)
    {
        // Wait until button is pressed
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay(500000);

        // Enable the SPI2 Peripheral
        SPI_PeriClockControl(SPI2, ENABLE);

        // Send length of data
        SPI_SendData(SPI2, &length, 1);

        // Send data
        SPI_SendData(SPI2, (uint8_t *)data, length);

        // Confirm SPI2 not busy
        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

        // Disable the SPI2 Peripheral
        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}