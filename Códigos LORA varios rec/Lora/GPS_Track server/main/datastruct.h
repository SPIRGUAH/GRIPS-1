#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include "Arduino.h"

// Definición de la estructura de datos
typedef struct {
    byte TX_ID;
    byte RX_ID;
    int seq;
    float latitude;
    float longitude;
    float altitude;
    float hdop;
    float temperature;
    float pressure;
    float humidity;
    float baro_altitude;
    float ext_temperature_ours;
    unsigned int CP10Sec_Gravity;
    // unsigned int CP10Sec_Libellium;
} Data;

// Declarar una instancia global (extern evita la redefinición)
extern Data data;

// Declaro la instancia global 
extern char t_buf[16];


#endif // DATASTRUCT_H
