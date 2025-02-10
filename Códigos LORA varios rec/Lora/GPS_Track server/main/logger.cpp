/*
#include "FS.h"
#include <LittleFS.h>
#include "logger.h"

static u_int8_t buf[FLASH_BLOCK_SIZE];
static int line = 0;

int write_buf()
{
    File file = LittleFS.open(LOGFILE, FILE_APPEND); 
    if(!file)
    {
        Serial.println("Failed to open file for appending");
        return -1;
    }
    file.write(buf, sizeof(buf));
    file.close();
    return 0;
}

bool logger(char * entry)
{
    memcpy(&buf[line*LINE_SIZE], entry, LINE_SIZE);
    line++;
    if (line == FLASH_BLOCK_SIZE/LINE_SIZE)
    {
        write_buf();
        line=0;
    }
    return true;
}
*/

#include "FS.h"
#include <LittleFS.h>
#include "logger.h"

#define EXPECTED_COLUMNS 10 // Número esperado de columnas en la trama

static u_int8_t buf[FLASH_BLOCK_SIZE];
static int line = 0;

int write_buf()
{
    File file = LittleFS.open(LOGFILE, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return -1;
    }
    file.write(buf, sizeof(buf));
    file.close();
    return 0;
}

// Función para validar el formato de una trama
bool validate_entry(const char *entry)
{
    String entryStr = String(entry);

    // Contar el número de columnas separadas por comas
    int columnCount = 0;
    for (int i = 0; i < entryStr.length(); i++)
    {
        if (entryStr[i] == ',')
        {
            columnCount++;
        }
    }

    // Verificar si el número de columnas coincide con el esperado
    if (columnCount != EXPECTED_COLUMNS - 1) // Comas son columnas - 1
    {
        Serial.println("[ERROR] Trama inválida: número de columnas incorrecto");
        return false;
    }

    // Verificar que no haya caracteres extraños (opcional, basado en formato esperado)
    for (int i = 0; i < entryStr.length(); i++)
    {
        if (!isPrintable(entryStr[i]) && entryStr[i] != '\n')
        {
            Serial.println("[ERROR] Trama inválida: contiene caracteres no imprimibles");
            return false;
        }
    }

    return true;
}

bool logger(char *entry)
{
    // Validar la trama antes de procesarla
    if (!validate_entry(entry))
    {
        return false; // No registrar tramas inválidas
    }

    memcpy(&buf[line * LINE_SIZE], entry, LINE_SIZE);
    line++;
    if (line == FLASH_BLOCK_SIZE / LINE_SIZE)
    {
        write_buf();
        Serial.print ("Block sent");
        line = 0;
    }
    return true;
}



