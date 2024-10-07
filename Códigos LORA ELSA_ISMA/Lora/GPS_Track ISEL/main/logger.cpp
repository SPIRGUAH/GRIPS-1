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



