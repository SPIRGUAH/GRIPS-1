#ifndef LOG_ENTRY_TESTS_H
#define LOG_ENTRY_TESTS_H

#include "Arduino.h"
#include "logger.h"
#include <TinyGPS++.h>
#include "LittleFS.h"
#include "datastruct.h"


// 📌 Prototipos de funciones para las diferentes pruebas
void LogEntryOriginal(char *log_entry, Data data, char *t_buf);
void LogEntryWithTruncateCheck(char *log_entry, Data data, char *t_buf);
void LogEntryWithSpaces(char *log_entry, Data data, char *t_buf);
void LogEntryWithInterruptProtection(char *log_entry, Data data, char *t_buf);

// 📌 Función para ejecutar todas las pruebas
void TestAllLogEntryVersions(char *log_entry, Data data, char *t_buf);

#endif // LOG_ENTRY_TESTS_H
