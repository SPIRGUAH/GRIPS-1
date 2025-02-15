#include "DataStruct.h"

// Inicialización de la estructura con los valores deseados
Data data = {
    .TX_ID = 0x55,
    .RX_ID = 0xAA,
    .seq = 0,
    .latitude = 0.0,
    .longitude = 0.0,
    .altitude = 0.0,
    .hdop = 0.0,
    .temperature = 0.0,
    .pressure = 0.0,
    .humidity = 0.0,
    .baro_altitude = 0.0,
    .ext_temperature_ours = 0.0,
    .CP10Sec_Gravity = 0
    // .CP10Sec_Libellium = 0 // Descomentar si es necesario
};

char t_buf[16];