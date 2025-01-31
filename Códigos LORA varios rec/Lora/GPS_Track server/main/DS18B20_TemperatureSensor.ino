#include <OneWire.h>
#include <DallasTemperature.h>

// Pin donde está conectado el DS18B20
#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Función de setup para el sensor DS18B20
void DS18B20_setup(void) {
    sensors.begin();
}

// Función para obtener la temperatura en grados Celsius
float DS18B20_tempC(void) {
    sensors.requestTemperatures(); // Solicitar temperatura
    return sensors.getTempCByIndex(0); // Obtener la temperatura
}
