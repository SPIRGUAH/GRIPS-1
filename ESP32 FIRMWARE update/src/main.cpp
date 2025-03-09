#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial gpsSerial(0);  // UART1 para GPS

void setup() {
    Serial.begin(115200);  // Comunicación con la PC por USB
    gpsSerial.begin(9600, SERIAL_8N1, 1, 3);  // TX=12, RX=15 (pines correctos en TTGO T-Beam)
}

void loop() {
    // Passthrough: Reenviar datos del GPS a la PC
    while (gpsSerial.available()) {
        Serial.write(gpsSerial.read());
        Serial.print(" CPM | Geiger 2: ");
        Serial.println(gpsSerial.read());
    }

    // Permitir enviar datos desde la PC al GPS (para flasheo en u-center)
    while (Serial.available()) {
        gpsSerial.write(Serial.read());
    }
}
