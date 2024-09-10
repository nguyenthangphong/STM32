#include <HardwareSerial.h>

/* UART2 */
#define TXD2_PIN    17
#define RXD2_PIN    16

HardwareSerial SerialPort(2);

void setup()
{
    SerialPort.begin(115200, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
    Serial.begin(9600);
    Serial.println("\nESP32 setup done...");
}

void loop()
{
    int status = SerialPort.available();

    if (status)
    {
        Serial.println("\nUART2 ESP32 Ready...");

        int length = status;

        char buffer[length];
        SerialPort.readBytes(buffer, length);
        buffer[length] = '\0';
        
        Serial.print("buffer: ");
        Serial.println(buffer);
    }
    else
    {
        Serial.println("\nUART2 ESP32 Not Ready...");
    }

    delay(1000);
}