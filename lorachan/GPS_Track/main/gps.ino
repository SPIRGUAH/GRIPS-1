/*

  GPS module

  Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <TinyGPS++.h>

uint32_t LatitudeBinary;
uint32_t LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t sats;
char t[32]; // used to sprintf for Serial output

TinyGPSPlus _gps;
HardwareSerial _serial_gps(GPS_SERIAL_NUM);

void AIBChecksum(char *message, size_t length)
{
    uint8_t CK_A = 0, CK_B = 0;

    // Calculate the checksum, starting from the CLASS field (which is message[2])
    for (size_t i = 2; i < length - 2; i++) {
        CK_A = (CK_A + message[i]) & 0xFF;
        CK_B = (CK_B + CK_A) & 0xFF;
    }

    // Place the calculated checksum values in the message
    message[length - 2] = CK_A;
    message[length - 1] = CK_B;
}

void gps_time(char * buffer, uint8_t size) {
    snprintf(buffer, size, "%02d:%02d:%02d", _gps.time.hour(), _gps.time.minute(), _gps.time.second());
}

float gps_latitude() {
    return _gps.location.lat();
}

float gps_longitude() {
    return _gps.location.lng();
}

float gps_altitude() {
    return _gps.altitude.meters();
}

float gps_hdop() { 
    return _gps.hdop.hdop();
}

uint8_t gps_sats() {
    return _gps.satellites.value();
}

void gps_setup() {
static char PROGMEM airborne[44] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 
                              0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                            };    

    _serial_gps.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    AIBChecksum(airborne, 44);
    _serial_gps.write(airborne, 44);
    delay(1000);
}

static void gps_loop() {
    while (_serial_gps.available()) {
        _gps.encode(_serial_gps.read());
    }
}

#if defined(PAYLOAD_USE_FULL)

    // More data than PAYLOAD_USE_CAYENNE
    void buildPacket(uint8_t txBuffer[10])
    {
        LatitudeBinary = ((_gps.location.lat() + 90) / 180.0) * 16777215;
        LongitudeBinary = ((_gps.location.lng() + 180) / 360.0) * 16777215;
        altitudeGps = _gps.altitude.meters();
        hdopGps = _gps.hdop.value() / 10;
        sats = _gps.satellites.value();

        sprintf(t, "Lat: %f", _gps.location.lat());
        Serial.println(t);
        sprintf(t, "Lng: %f", _gps.location.lng());
        Serial.println(t);
        sprintf(t, "Alt: %d", altitudeGps);
        Serial.println(t);
        sprintf(t, "Hdop: %d", hdopGps);
        Serial.println(t);
        sprintf(t, "Sats: %d", sats);
        Serial.println(t);

        txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
        txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
        txBuffer[2] = LatitudeBinary & 0xFF;
        txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
        txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
        txBuffer[5] = LongitudeBinary & 0xFF;
        txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
        txBuffer[7] = altitudeGps & 0xFF;
        txBuffer[8] = hdopGps & 0xFF;
        txBuffer[9] = sats & 0xFF;
    }

#elif defined(PAYLOAD_USE_CAYENNE)

    // CAYENNE DF
    void buildPacket(uint8_t txBuffer[11])
    {
        sprintf(t, "Lat: %f", _gps.location.lat());
        Serial.println(t);
        sprintf(t, "Lng: %f", _gps.location.lng());
        Serial.println(t);        
        sprintf(t, "Alt: %f", _gps.altitude.meters());
        Serial.println(t);        
        int32_t lat = _gps.location.lat() * 10000;
        int32_t lon = _gps.location.lng() * 10000;
        int32_t alt = _gps.altitude.meters() * 100;
        
        txBuffer[2] = lat >> 16;
        txBuffer[3] = lat >> 8;
        txBuffer[4] = lat;
        txBuffer[5] = lon >> 16;
        txBuffer[6] = lon >> 8;
        txBuffer[7] = lon;
        txBuffer[8] = alt >> 16;
        txBuffer[9] = alt >> 8;
        txBuffer[10] = alt;
    }

#endif
