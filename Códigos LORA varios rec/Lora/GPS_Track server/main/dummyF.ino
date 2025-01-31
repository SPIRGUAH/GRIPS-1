// Función para generar latitud aleatoria
float gps_latitude_dummy() {
  return random(-90, 90) + random(0, 100) / 100.0; // Latitud entre -90 y 90 grados
}

// Función para generar longitud aleatoria
float gps_longitude_dummy() {
  return random(-180, 180) + random(0, 100) / 100.0; // Longitud entre -180 y 180 grados
}

// Función para generar altitud aleatoria
float gps_altitude_dummy() {
  return random(0, 40000); // Altitud entre 0 y 40,000 metros
}

// Función para generar HDOP aleatorio
float gps_hdop_dummy() {
  return random(1, 10) + random(0, 100) / 100.0; // HDOP entre 1.0 y 10.0
}

// Función para generar temperatura aleatoria (BME280)
float bme280_temperature_dummy() {
  return random(-40, 85) + random(0, 100) / 100.0; // Temperatura entre -40 y 85 grados Celsius
}

// Función para generar presión aleatoria (BME280)
float bme280_pressure_dummy() {
  return random(0, 1100); // Presión entre 0 y 1100 hPa
}

// Función para generar humedad aleatoria (BME280)
float bme280_humidity_dummy() {
  return random(0, 100); // Humedad entre 0 y 100%
}

// Función para generar altitud barométrica aleatoria (BME280)
float bme280_baro_altitude_dummy() {
  return random(0, 40000); // Altitud barométrica entre 0 y 40,000 metros
}

// Función para generar temperatura externa aleatoria (DS18B20)
float DS18B20_tempC_dummy() {
  return random(-55, 125) + random(0, 100) / 100.0; // Temperatura entre -55 y 125 grados Celsius
}

// Función para generar recuento de pulsos (CP10Sec) aleatorio
int getCP10Sec_dummy() {
  return random(0, 100); // Recuento de pulsos entre 0 y 100
}
