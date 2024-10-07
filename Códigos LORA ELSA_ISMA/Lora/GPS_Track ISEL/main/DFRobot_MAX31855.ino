#include <DFRobot_MAX31855.h>
#include <Wire.h>

DFRobot_MAX31855 max31855(&Wire, 0x10);

void DFRobot_max31855_setup (void) {
    max31855.begin();
}

float DFRobot_max31855_temperature()
{
    return max31855.readCelsius();
}
