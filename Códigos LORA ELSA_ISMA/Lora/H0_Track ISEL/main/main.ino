/*

  Main module

  # Modified by Kyle T. Gabriel to fix issue with incorrect GPS data for TTNMapper

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

#include "configuration.h"
#include "rom/rtc.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>

#define PORT 5055
#define BALLOON_ID 1234
#define RECEIVER_ID 4321
#define TRACCAR_SRV "http://srs-copper.duckdns.org"

#include "axp20x.h"
AXP20X_Class axp;
bool pmu_irq = false;
String baChStatus = "No charging";

bool ssd1306_found = false;
bool axp192_found = false;

bool packetSent, packetQueued;
int counter = 0; //Contador de mensajes

HTTPClient http;

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause;  // the reason we booted this time

//WiFi connection
const char* ssid     = "S2P";     // change this for your own network
//const char* password = "TExY-yAx7-xwMO-Fh6g";  // change this for your own network
const char* password = "PASSWORD";  // change this for your own network
//const char* ssid     = "Alifanfaron";     // change this for your own network
//const char* password = "MZRSSP_1968";  // change this for your own network

WiFiClient espClient;
PubSubClient client(espClient);

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
    unsigned int CP10Sec;
} Data;
// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------

void buildPacket(uint8_t txBuffer[]);  // needed for platformio

/**
 * If we have a valid position send it to the server.
 */
int LoRaReceive() 
{
    char buf[128];
    float latitude;
    float longitude;
    float altitude;
    float hdop;
    float temperature;
    float pressure;
    float humidity;
    float baro_altitude;
    float ext_temperature_ours;
    unsigned int CP10Sec;

    Data data;

    // try to parse packet
    int packetSize = LoRa.parsePacket();


    if (packetSize) 
    {
        // received a packet
        Serial.print("Received packet '");
        Serial.print("Packetsize: ");
        Serial.println (packetSize);
        LoRa.readBytes((byte *)&data, packetSize);

        for (int i = 0; i < packetSize; i++) {
            Serial.print(' ');
            Serial.print(((byte *)&data)[i]);
        }
        Serial.println();

        if ((data.TX_ID !=0x55) || (data.RX_ID != 0xAA))
        {
            Serial.println("Noise received...");
            return -1;
        }    

        longitude      = data.longitude;
        latitude       = data.latitude;
        altitude       = data.altitude;
        hdop           = data.hdop;
        temperature    = data.temperature;
        pressure       = data.pressure;
        humidity       = data.humidity;
        baro_altitude  = data.baro_altitude;
        ext_temperature_ours = data.ext_temperature_ours;
        CP10Sec        = data.CP10Sec;

        Serial.print("Latitude: ");
        Serial.println(latitude);
        Serial.print("Longitude: ");
        Serial.println(longitude);        
        Serial.print("Altitude: ");
        Serial.println(altitude);
        Serial.print("Hdop: ");
        Serial.println(hdop);
        Serial.print("Temperature: ");
        Serial.println(temperature);
        Serial.print("Pressure: ");
        Serial.println(pressure);
        Serial.print("Humidity: ");
        Serial.println(humidity);
        Serial.print("Barometric altitude: ");
        Serial.println(baro_altitude);
        Serial.print("External Temperature: ");
        Serial.println(ext_temperature_ours);
        Serial.print("Geiger count: ");
        Serial.println(CP10Sec);

        // print RSSI of packet
        Serial.print("RSSI: ");
        Serial.println(LoRa.packetRssi());
        // print SNR
        Serial.print("SNR: ");
        Serial.println(LoRa.packetSnr());

        snprintf(buf, sizeof(buf), "RSSI: %i\n", LoRa.packetRssi());
        screen_print(buf);

        snprintf(buf, sizeof(buf), "SNR: %.1f\n", LoRa.packetSnr());
        screen_print(buf);

        snprintf(buf, sizeof(buf), "Latitude: %3.6f\n", data.latitude);
        screen_print(buf);

        snprintf(buf, sizeof(buf), "Longitude: %3.6f\n", data.longitude);
        screen_print(buf);
        
        snprintf(buf, sizeof(buf), "Altitude: %3.6f\n", data.altitude);
        screen_print(buf);

        //Send MQTT packet
        if ((altitude > 0) && (altitude < 600000))
        {
            sprintf(buf, "%3.6f", altitude);  
            client.publish("globo/altitude", buf);
        }       
        snprintf(buf, sizeof(buf), "%i", LoRa.packetRssi());
        client.publish("globo/RSSI", buf);
        sprintf(buf, "%3.6f", hdop);  
        client.publish("globo/hdop", buf);
        if ((ext_temperature_ours < 80) && (ext_temperature_ours > -80))
        { 
            sprintf(buf, "%3.6f", ext_temperature_ours);  
            client.publish("globo/ext_temperature", buf);
        }
        if ((temperature < 80) && (temperature > -80))
        { 
            sprintf(buf, "%3.6f", temperature);  
            client.publish("globo/temperature", buf);
        }
        if ((pressure < 1100) && (pressure > 0))
        {    
            sprintf(buf, "%3.6f", pressure);  
            client.publish("globo/pressure", buf);
        }    
        if ((humidity > 0) && (humidity < 100))
        {
            sprintf(buf, "%3.6f", humidity);  
            client.publish("globo/humidity", buf);
        }    
        if ((baro_altitude > 0) && (baro_altitude < 600000))
        {
            sprintf(buf, "%3.6f", baro_altitude);  
            client.publish("globo/baro_altitude", buf);
        }
        if (CP10Sec >= 0)
        {
            sprintf(buf, "%d", CP10Sec);  
            client.publish("globo/CP10Sec", buf);
        }

        //Send Traccar Balloon data
        if (altitude > 0)
        {
            sprintf(buf, "%s:%d/?id=%d&lat=%3.5f&lon=%3.5f&hdop=%3.2f&altitude=%3.5f&speed=5", TRACCAR_SRV, PORT, BALLOON_ID, latitude, longitude, hdop, altitude);
            http.begin(buf); 
            int httpCode = http.GET();                                       
 
            if (httpCode == 200) { //Check for the returning code
                Serial.println("Balloon data send to the server");   
            } else {
                Serial.println("Error on HTTP request");
            }
            http.end(); //Free the resources
        }
        
        //Send Traccar Receiver data
        if (gps_altitude() > 0)
        {
            sprintf(buf, "%s:%d/?id=%d&lat=%3.5f&lon=%3.5f&hdop=%3.2f&altitude=%3.5f&speed=5", TRACCAR_SRV, PORT, RECEIVER_ID, gps_latitude(), gps_longitude(), gps_hdop(), gps_altitude());
            http.begin(buf); 
            int httpCode = http.GET();                                       
 
            if (httpCode == 200) { //Check for the returning code
                Serial.println("Receiver data send to the server");   
            } else {
                Serial.println("Error on HTTP request");
            }
            http.end(); //Free the resources
        }    
    }    
    return 0;
}

//Call back for MQTT topic registration
void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//MQTT reconnect
void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("H0")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void doDeepSleep(uint64_t msecToWake)
{
    Serial.printf("Entering deep sleep for %llu seconds\n", msecToWake / 1000);

    // not using wifi yet, but once we are this is needed to shutoff the radio hw
    // esp_wifi_stop();

    screen_off();  // datasheet says this will draw only 10ua
    LMIC_shutdown();  // cleanly shutdown the radio
    
    if(axp192_found) {
        // turn on after initial testing with real hardware
        axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // LORA radio
        axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS main power
    }

    // FIXME - use an external 10k pulldown so we can leave the RTC peripherals powered off
    // until then we need the following lines
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Only GPIOs which are have RTC functionality can be used in this bit map: 0,2,4,12-15,25-27,32-39.
    uint64_t gpioMask = (1ULL << BUTTON_PIN);

    // FIXME change polarity so we can wake on ANY_HIGH instead - that would allow us to use all three buttons (instead of just the first)
    gpio_pullup_en((gpio_num_t) BUTTON_PIN);

    esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);

    esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL);  // call expects usecs
    esp_deep_sleep_start();                               // TBD mA sleep current (battery)
}


void sleep() {
#if SLEEP_BETWEEN_MESSAGES

    // If the user has a screen, tell them we are about to sleep
    if (ssd1306_found) {
        // Show the going to sleep message on the screen
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Sleeping in %3.1fs\n", (MESSAGE_TO_SLEEP_DELAY / 1000.0));
        screen_print(buffer);

        // Wait for MESSAGE_TO_SLEEP_DELAY millis to sleep
        delay(MESSAGE_TO_SLEEP_DELAY);

        // Turn off screen
        screen_off();
    }

    // Set the user button to wake the board
    sleep_interrupt(BUTTON_PIN, LOW);

    // We sleep for the interval between messages minus the current millis
    // this way we distribute the messages evenly every SEND_INTERVAL millis
    uint32_t sleep_for = (millis() < SEND_INTERVAL) ? SEND_INTERVAL - millis() : SEND_INTERVAL;
    doDeepSleep(sleep_for);

#endif
}

void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == SSD1306_ADDRESS) {
                ssd1306_found = true;
                Serial.println("ssd1306 display found");
            }
            if (addr == AXP192_SLAVE_ADDRESS) {
                axp192_found = true;
                Serial.println("axp192 PMU found");
            }
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

/**
 * Init the power manager chip
 * 
 * axp192 power 
    DCDC1 0.7-3.5V @ 1200mA max -> OLED  // If you turn this off you'll lose comms to the axp192 because the OLED and the axp192 share the same i2c bus, instead use ssd1306 sleep mode
    DCDC2 -> unused
    DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
    LDO1 30mA -> charges GPS backup battery  // charges the tiny J13 battery by the GPS to power the GPS ram (for a couple of days), can not be turned off
    LDO2 200mA -> LORA
    LDO3 200mA -> GPS
 */

void axp192Init() {
    if (axp192_found) {
        if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
            Serial.println("AXP192 Begin PASS");
        } else {
            Serial.println("AXP192 Begin FAIL");
        }
        // axp.setChgLEDMode(LED_BLINK_4HZ);
        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
        Serial.println("----------------------------------------");

        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // LORA radio
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS main power
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
        axp.setDCDC1Voltage(3300);  // for the OLED power

        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
            pmu_irq = true;
        }, FALLING);

        axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
        axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
        axp.clearIRQ();

        if (axp.isCharging()) {
            baChStatus = "Charging";
        }
    } else {
        Serial.println("AXP192 not found");
    }
}


// Perform power on init that we do on each wake from deep sleep
void initDeepSleep() {
    bootCount++;
    wakeCause = esp_sleep_get_wakeup_cause(); 
    /* 
    Not using yet because we are using wake on all buttons being low

    wakeButtons = esp_sleep_get_ext1_wakeup_status();        // If one of these buttons is set it was the reason we woke
    if (wakeCause == ESP_SLEEP_WAKEUP_EXT1 && !wakeButtons)  // we must have been using the 'all buttons rule for waking' to support busted boards, assume button one was pressed
        wakeButtons = ((uint64_t)1) << buttons.gpios[0];
    */

    Serial.printf("booted, wake cause %d (boot count %d)\n", wakeCause, bootCount);
}


void setup()
{

    // Debug
    #ifdef DEBUG_PORT
        DEBUG_PORT.begin(SERIAL_BAUD);
    #endif

    initDeepSleep();

    Wire.begin(I2C_SDA, I2C_SCL);
    scanI2Cdevice();

    axp192Init();

    // Buttons & LED
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    #ifdef LED_PIN
        pinMode(LED_PIN, OUTPUT);
    #endif

    // Hello
    DEBUG_MSG(APP_NAME " " APP_VERSION "\n");

    // Don't init display if we don't have one or we are waking headless due to a timer event
    if (wakeCause == ESP_SLEEP_WAKEUP_TIMER)
        ssd1306_found = false;	// forget we even have the hardware

    if (ssd1306_found) screen_setup();

    // Init GPS
    gps_setup();

    Serial.println("LoRa Receiver");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQ)) 
    {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    // Show logo on first boot after removing battery
    #ifndef ALWAYS_SHOW_LOGO
    if (bootCount == 0) {
    #endif
        screen_print(APP_NAME " " APP_VERSION, 0, 0);
        screen_show_logo();
        screen_update();
        delay(LOGO_DELAY);
    #ifndef ALWAYS_SHOW_LOGO
    }
    #endif

    //WiFi initialization
    Serial.println("Connecting To  Wifi");
    do {
        WiFi.begin(ssid, password);
        Serial.print(".");
        delay (1000);
    } while(WiFi.status() != WL_CONNECTED);
    
    Serial.println("WIFI OK");
   
    //MQTT server
    client.setServer("srs-copper.duckdns.org", 1883);
    client.setCallback(callback);
}

void loop() {
    gps_loop();
    //ttn_loop();
    screen_loop();

    if (packetSent) {
        packetSent = false;
        sleep();
    }

    // if user presses button for more than 3 secs, discard our network prefs and reboot (FIXME, use a debounce lib instead of this boilerplate)
    static bool wasPressed = false;
    static uint32_t minPressMs;  // what tick should we call this press long enough
    if (!digitalRead(BUTTON_PIN)) {
        if (!wasPressed) {
            // just started a new press
            Serial.println("pressing");
            wasPressed = true;
            minPressMs = millis() + 3000;
        }
    }
    else if (wasPressed) {
        // we just did a release
        wasPressed = false;
        if (millis() > minPressMs) {
            // held long enough
            #ifndef PREFS_DISCARD
                screen_print("Discarding prefs disabled\n");
            #endif
        }
    }
    LoRaReceive();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
