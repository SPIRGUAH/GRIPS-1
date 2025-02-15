#include "LogEntryTests.h"

void LogEntryOriginal(char *log_entry, Data data, char *t_buf) {
    memset(log_entry, 0, LINE_SIZE);
    snprintf(log_entry, LINE_SIZE, "%s,%4.2f,%3.2f,%3.2f,%2.2f,%5.2f,%4.6f,%4.6f,%d,\n",
                t_buf, data.pressure, data.ext_temperature_ours, data.temperature,
                data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec_Gravity);
    logger(log_entry);
}

/*
char LogEntryOriginal(*char,*Data){

    memset(log_entry, 0, LINE_SIZE);
    //snprintf(log_entry, LINE_SIZE, "%05i: 00, 01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13 \n", data.seq);
    gps_time(t_buf, sizeof(t_buf));
    //printf("%s, %4.2f, %3.2f, %3.2f, %2.2f, %5.2f, %4.6f, %4.6f, %d\n", t_buf, data.pressure, data.ext_temperature_ours, data.temperature, data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec);
    snprintf(log_entry, LINE_SIZE, "%s,%4.2f,%3.2f,%3.2f,%2.2f,%5.2f,%4.6f,%4.6f, %d,\n", t_buf, data.pressure, data.ext_temperature_ours, data.temperature, data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec);

}  
*/
 
    /*
    memset(log_entry, 0, LINE_SIZE);
    //snprintf(log_entry, LINE_SIZE, "%05i: 00, 01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13 \n", data.seq);
    gps_time(t_buf, sizeof(t_buf));
    printf("%s, %4.2f, %3.2f, %3.2f, %2.2f, %5.2f, %4.6f, %4.6f, %d\n", t_buf, data.pressure, data.ext_temperature_ours, data.temperature, data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec);
    int len = snprintf(log_entry, LINE_SIZE, "%s,%4.2f,%3.2f,%3.2f,%2.2f,%5.2f,%4.6f,%4.6f, %d\n", t_buf, data.pressure, data.ext_temperature_ours, data.temperature, data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec);
    if (len >= LINE_SIZE) {  
        Serial.printf("[ERROR] Buffer demasiado pequeño: %d bytes escritos en un buffer de %d bytes\n", len, LINE_SIZE);
    }else{
        memset(log_entry + len, ' ', LINE_SIZE - 1 - len); // Rellenar con espacios
    }
    log_entry[LINE_SIZE - 1] = '\0'; // Asegurar terminación    
    */
// 📌 Versión con Manejo de Truncamiento
void LogEntryWithTruncateCheck(char *log_entry, Data data, char *t_buf) {
    memset(log_entry, 0, LINE_SIZE);
    int len = snprintf(log_entry, LINE_SIZE, "%s,%4.2f,%3.2f,%3.2f,%2.2f,%5.2f,%4.6f,%4.6f,%d,\n",
                       t_buf, data.pressure, data.ext_temperature_ours, data.temperature,
                       data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec_Gravity);
    if (len >= LINE_SIZE) {
        Serial.printf("[ERROR] Buffer demasiado pequeño: %d bytes escritos en un buffer de %d bytes\n", len, LINE_SIZE);
        log_entry[LINE_SIZE - 1] = '\0'; // Forzar terminación segura
    }
    //Serial.printf("DEBUG (TruncateCheck): [%s]\n", log_entry);
    logger(log_entry);
}

// 📌 Versión con Espacios en el Buffer
void LogEntryWithSpaces(char *log_entry, Data data, char *t_buf) {
    memset(log_entry, ' ', LINE_SIZE - 1);
    log_entry[LINE_SIZE - 1] = '\0';
    int len = snprintf(log_entry, LINE_SIZE, "%s,%4.2f,%3.2f,%3.2f,%2.2f,%5.2f,%4.6f,%4.6f,%d,\n",
                        t_buf, data.pressure, data.ext_temperature_ours, data.temperature,
                        data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec_Gravity);
    if (len >= LINE_SIZE) {
        Serial.printf("[ERROR] Buffer demasiado pequeño: %d bytes escritos en un buffer de %d bytes\n", len, LINE_SIZE);
        log_entry[LINE_SIZE - 1] = '\0';
    } else {
        memset(log_entry + len, ' ', LINE_SIZE - 1 - len);
        log_entry[LINE_SIZE - 1] = '\0';
    }
    //Serial.printf("DEBUG (WithSpaces): [%s]\n", log_entry);
    logger(log_entry);
}

// 📌 Versión con Protección contra Interrupciones
void LogEntryWithInterruptProtection(char *log_entry, Data data, char *t_buf) {
    memset(log_entry, 0, LINE_SIZE);
    snprintf(log_entry, LINE_SIZE, "%s,%4.2f,%3.2f,%3.2f,%2.2f,%5.2f,%4.6f,%4.6f,%d,\n",
                t_buf, data.pressure, data.ext_temperature_ours, data.temperature,
                data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec_Gravity);
    noInterrupts(); // Deshabilitar interrupciones
    logger(log_entry);
    interrupts(); // Habilitar interrupciones nuevamente
    //Serial.printf("DEBUG (InterruptProtected): [%s]\n", log_entry);
}

    /*
    // Inicializar el buffer con espacios
    memset(log_entry, ' ', LINE_SIZE - 1);
    log_entry[LINE_SIZE - 1] = '\0'; // Terminar correctamente el buffer

    // Generar la marca de tiempo en t_buf
    gps_time(t_buf, sizeof(t_buf));

    // Usar snprintf para escribir en log_entry
    int len = snprintf(log_entry, LINE_SIZE, "%s,%4.2f,%3.2f,%3.2f,%2.2f,%5.2f,%4.6f,%4.6f,%d\n",
                    t_buf, data.pressure, data.ext_temperature_ours, data.temperature,
                    data.humidity, data.altitude, data.longitude, data.latitude, data.CP10Sec_Gravity);

    // Validar si snprintf truncó la salida
    if (len >= LINE_SIZE) {
        Serial.printf("[ERROR] Buffer demasiado pequeño: %d bytes escritos en un buffer de %d bytes\n", len, LINE_SIZE);
        log_entry[LINE_SIZE - 1] = '\0'; // Terminar correctamente para evitar errores
    } else {
        // Rellenar cualquier espacio restante con espacios (evita residuos de \x00)
        if (len < LINE_SIZE - 1) {
            memset(log_entry + len, ' ', LINE_SIZE - 1 - len);
        }
        log_entry[LINE_SIZE - 1] = '\0'; // Terminar correctamente
    }
    */

// 📌 Función de prueba para ejecutar todas las versiones
void TestAllLogEntryVersions(char *log_entry, Data data, char *t_buf) {

    Serial.println("===== Ejecutando Pruebas de Logging =====");
    
    Serial.println("▶ Prueba 1: LogEntryOriginal");
    LogEntryOriginal(log_entry, data, t_buf);

    Serial.println("▶ Prueba 2: LogEntryWithTruncateCheck");
    LogEntryWithTruncateCheck(log_entry, data, t_buf);

    Serial.println("▶ Prueba 3: LogEntryWithSpaces");
    LogEntryWithSpaces(log_entry, data, t_buf);

    Serial.println("▶ Prueba 4: LogEntryWithInterruptProtection");
    LogEntryWithInterruptProtection(log_entry, data, t_buf);

    Serial.println("===== Fin de Pruebas =====");
}
    