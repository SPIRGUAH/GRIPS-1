#include "configuration.h"

void lora_setup(void)
{
    Serial.println("LoRa Sender");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQ)) 
    {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
}  
