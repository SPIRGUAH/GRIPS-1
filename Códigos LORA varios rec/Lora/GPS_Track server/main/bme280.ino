#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_I2C_ADDR 0x76

Adafruit_BME280 bme;

void bme280_setup(void)
{
    Serial.println("BME280 setup");
    if (!bme.begin(BME_I2C_ADDR)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
	}else{
        bme280_found = true;
    }
}  

float bme280_pressure()
{
    return bme.readPressure() / 100.0F;
}

float bme280_temperature()
{
    return bme.readTemperature();
}

float bme280_baro_altitude()
{
    return bme.readAltitude(SEALEVELPRESSURE_HPA);
}

float bme280_humidity()
{
    return bme.readHumidity();
}