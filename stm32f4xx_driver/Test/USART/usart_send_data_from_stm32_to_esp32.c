#include "stm32f411xx.h"
#include "stm32f411xx_usart_driver.h"
#include "stm32f411xx_gpio_driver.h"
#include "delay.h"
#include <string.h>

char msg[1024] = "USART Tx testing...\n\r";

st_USART_Handle_t USART2_Handle;
st_GPIO_Handle_t USART2_GPIO_Handle, USART2_GPIO_BTN_Handle, USART2_GPIO_LED_Handle;

void USART2_Init(void);
void USART2_GPIO_Init(void);
void USART2_BTN_Init(void);
void USART2_LED_Init(void);

int main(void)
{
    USART2_BTN_Init();
    USART2_LED_Init();
    USART2_GPIO_Init();
    USART2_Init();

    USART_PeriClockControl(USART2, ENABLE);

    while (1)
    {
        // while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
        delay(500000);
        USART_SendData(&USART2_Handle, (uint8_t*)msg, strlen(msg));
        delay(500000);
    }

    return 0;
}

void USART2_Init(void)
{
    USART2_Handle.pUSARTx = USART2;
    USART2_Handle.USART_Config.USART_Baud = USART_BAUD_9600;
    USART2_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    USART2_Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    USART2_Handle.USART_Config.USART_NoOfStopBit = USART_STOP_BITS_1;
    USART2_Handle.USART_Config.USART_WordLength = USART_WORD_LENGTH_8BITS;
    USART2_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(&USART2_Handle);
}

void USART2_GPIO_Init(void)
{
    USART2_GPIO_Handle.pGPIOx = GPIOA;
    USART2_GPIO_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    USART2_GPIO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USART2_GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    USART2_GPIO_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    USART2_GPIO_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    /* USART2 TX */
    USART2_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&USART2_GPIO_Handle);

    /* USART2 RX */
    USART2_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&USART2_GPIO_Handle);
}

void USART2_BTN_Init(void)
{
    USART2_GPIO_BTN_Handle.pGPIOx = GPIOA;
    USART2_GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    USART2_GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    USART2_GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    USART2_GPIO_BTN_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&USART2_GPIO_BTN_Handle);
}

void USART2_LED_Init(void)
{
    USART2_GPIO_LED_Handle.pGPIOx = GPIOD;
    USART2_GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    USART2_GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    USART2_GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    USART2_GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    USART2_GPIO_LED_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&USART2_GPIO_LED_Handle);
}