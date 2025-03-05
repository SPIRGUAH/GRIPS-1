#include <Arduino.h>

#include <HardwareSerial.h>

HardwareSerial gpsSerial(1);  // UART1 para GPS

void setup() {
    Serial.begin(115200);  // Comunicación con la PC por USB
    gpsSerial.begin(9600, SERIAL_8N1, 12, 34);  // GPS en UART1 (TX=12, RX=34)
}

void loop() {
    // Passthrough: Reenviar datos del GPS a la PC
    while (gpsSerial.available()) {
        Serial.write(gpsSerial.read());
    }

    // Permitir enviar datos desde la PC al GPS (para flasheo en u-center)
    while (Serial.available()) {
        gpsSerial.write(Serial.read());
    }
}
