#include <SPI.h>
#include <HardwareSerial.h>

#define ESP32_SPI            3

#define ESP32_SPI_MOSI       23
#define ESP32_SPI_MISO       19
#define ESP32_SPI_SCLK       18
#define ESP32_SPI_CS         5

#define MAX                  500

SPIClass spi = SPIClass(ESP32_SPI);

char data[MAX];
uint8_t length = 0;
uint32_t i = 0;

void setup()
{
    Serial.begin(9600);
    spi.begin(ESP32_SPI_SCLK, ESP32_SPI_MISO, ESP32_SPI_MOSI, ESP32_SPI_CS);
    spi.setHwCs(false);
    
    Serial.println("SPI Slave Initialized...");
    Serial.println("ESP32 Setup Done...");
}

void loop()
{
    Serial.println("Waiting for CS to go LOW...");

    while (digitalRead(ESP32_SPI_CS));

    Serial.println("CS LOW, Starting communication...");

    i = 0;
    length = spi.transfer(0x00);

    // for (int i = 0; i < length; i++)
    // {
    //     uint8_t buffer = ESP32_SPI_SlaveReceive();
    //     data[i] = buffer;

    //     Serial.print("i: ");
    //     Serial.println(i);

    //     Serial.print("buffer: ");
    //     Serial.println(buffer, HEX);
    // }

    // data[i] = '\0';

    Serial.print("length: ");
    Serial.println(length);

    // Serial.print("data: ");
    // Serial.println(data);

    Serial.println("Communication finished...");
}