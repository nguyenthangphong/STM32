#include "stm32f411xx.h"
#include "stm32f411xx_usart_driver.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"
#include <string.h>

#define LOW         0
#define BTN_PRESSED LOW

char msg[1024] = "USART Tx testing...\n\r";

st_USART_Handle_t USART2_Handle;

void USART2_Init(void);
void GPIO_USART2_Init(void);
void GPIO_BTN_Init(void);
void GPIO_LED_Init(void);

int main(void)
{
    GPIO_BTN_Init();

    GPIO_LED_Init();

    GPIO_USART2_Init();

    USART2_Init();

    USART_PeriClockControl(USART2, ENABLE);

    while (1)
    {
        while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED);
        delay(500000);

        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        delay(500000);

        USART_SendData(&USART2_Handle, (uint8_t*)msg, strlen(msg));
        delay(500000);
    }

    return 0;
}

void USART2_Init(void)
{
    memset(&USART2_Handle, 0, sizeof(USART2_Handle));

    USART2_Handle.pUSARTx = USART2;
    USART2_Handle.USART_Config.USART_Baud = USART_BAUD_115200;
    USART2_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    USART2_Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    USART2_Handle.USART_Config.USART_NoOfStopBit = USART_STOP_BITS_1;
    USART2_Handle.USART_Config.USART_WordLength = USART_WORD_LENGTH_8BITS;
    USART2_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(&USART2_Handle);
}

void GPIO_USART2_Init(void)
{
    st_GPIO_Handle_t GPIO_USART2_Handle;
    memset(&GPIO_USART2_Handle, 0, sizeof(GPIO_USART2_Handle));

    GPIO_USART2_Handle.pGPIOx = GPIOA;
    GPIO_USART2_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_USART2_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIO_USART2_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    GPIO_USART2_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_USART2_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    /* USART2 TX */
    GPIO_USART2_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&GPIO_USART2_Handle);

    /* USART2 RX */
    GPIO_USART2_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&GPIO_USART2_Handle);
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

void GPIO_LED_Init(void)
{
    st_GPIO_Handle_t GPIO_LED_Handle;
    memset(&GPIO_LED_Handle, 0, sizeof(GPIO_LED_Handle));

    GPIO_LED_Handle.pGPIOx = GPIOD;
    GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_LED_Handle);
}