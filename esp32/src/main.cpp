#include <SPI.h>
#include <HardwareSerial.h>

#define ESP32_SPI            3
#define ESP32_SPI_MOSI       23
#define ESP32_SPI_MISO       19
#define ESP32_SPI_SCLK       18
#define ESP32_SPI_CS         5
#define ESP32_SPI_MSBFIRST   1
#define ESP32_SPI_MODE0      0
#define ESP32_SPI_FREQUENCY  2000000
#define MAX                  500

SPIClass spi = SPIClass(ESP32_SPI);

char data[MAX];
uint16_t length = 0;
uint32_t i = 0;

void ESP32_SPI_SlaveInit(void);
uint8_t ESP32_SPI_SlaveReceive(void);
void ESP32_SPI_SlaveTransmit(char data);

void setup()
{
    Serial.begin(9600);
    ESP32_SPI_SlaveInit();
    Serial.println("\nESP32 Setup Done...");
}

void loop()
{
    Serial.println("\nWaiting...");

    while (digitalRead(ESP32_SPI_CS));

    Serial.println("\nStarting...");

    i = 0;

    length = ESP32_SPI_SlaveReceive();

    for (i = 0; i < length; i++)
    {
        data[i] = ESP32_SPI_SlaveReceive();
    }

    data[i] = '\0';

    Serial.print("data : ");
    Serial.println(data);

    Serial.print("length : ");
    Serial.println(length);
}

void ESP32_SPI_SlaveInit(void)
{
    pinMode(ESP32_SPI_SCLK, INPUT);
    pinMode(ESP32_SPI_MOSI, INPUT);
    pinMode(ESP32_SPI_MISO, OUTPUT);
    pinMode(ESP32_SPI_CS  , INPUT);
    spi.begin(ESP32_SPI_SCLK, ESP32_SPI_MISO, ESP32_SPI_MOSI, ESP32_SPI_CS);
}

uint8_t ESP32_SPI_SlaveReceive(void)
{   
    return spi.transfer(0x00);
}

void ESP32_SPI_SlaveTransmit(char data)
{
    spi.transfer(data);
}