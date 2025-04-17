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

#define PORT 5055   // Port to access the traccar server
#define BALLOON_ID 1234 // Beacon ID needed to track
#define RECEIVER_ID 4321 // ID of each receptor, can be named arbitrarily but needs to be unique in the same network
#define TRACCAR_SRV "http://srs-copper.duckdns.org" // URL of the traccar server where we locate all the payload and trackers

// Defined using AXP2102
#define XPOWERS_CHIP_AXP2101

#include "axp20x.h"
#include "XPowersLib.h"


AXP20X_Class axp_192;
XPowersPMU axp_2101;
bool pmu_irq = false;
String baChStatus = "No charging";

bool ssd1306_found = false;
bool axp192_found = false;
bool axp2101_found = false;
bool axpsecure = false;

bool packetSent, packetQueued;
int counter = 0; // Messages count

HTTPClient http;

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause;  // the reason we booted this time

//WiFi connection
const char* ssid     = "S2P";     // change this for your own network
const char* password = "PASSWORD";  // change this for your own network

WiFiClient espClient4321;
PubSubClient client(espClient4321); // Client ID sent to the server

typedef struct { // TM struct defined
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
int LoRaReceive()  // Function in charge of receiving and decoding the TM
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
        if ((altitude >= 0) && (altitude < 600000))
        {
            sprintf(buf, "%3.6f", altitude);  
            client.publish("globo/altitude", buf);
        }       
        snprintf(buf, sizeof(buf), "%i", LoRa.packetRssi());
        client.publish("globo/RSSI1", buf);
        sprintf(buf, "%3.6f", hdop);  
        client.publish("globo/hdop", buf);
        if ((ext_temperature_ours < 80) && (ext_temperature_ours > -128))
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
        sprintf(buf, "%3.5f", latitude);
        client.publish("globo/latitude", buf);
        sprintf(buf, "%3.5f", longitude);
        client.publish("globo/longitude",buf);

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
    if (client.connect("H1")) { // Client name needs to be unique for each network logged in the same MQTT port. Defined as HX where X is the number assigned to each receptor
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

void doDeepSleep(uint64_t msecToWake) // Function to save energy, currently unused
{
    Serial.printf("Entering deep sleep for %llu seconds\n", msecToWake / 1000);

    // not using wifi yet, but once we are this is needed to shutoff the radio hw
    // esp_wifi_stop();

    screen_off();  // datasheet says this will draw only 10ua
    LMIC_shutdown();  // cleanly shutdown the radio
    
    if(axp192_found) {
        // turn on after initial testing with real hardware
        axp_192.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // LORA radio
        axp_192.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS main axp_2101
    }
    /*
    if(axp2101_found) {
        // turn on after initial testing with real hardware
        axp_2101.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // LORA radio
        axp_2101.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS main axp_2101
    }
    */
    
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

void scanI2Cdevice(void) // Function to chech the direcctions I2C connected to the lora receiver
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
                Serial.println("axp PMU found");

                if (axp_2101.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
                    Serial.println("AXP2101 Begin PASS");
                    axp2101_found = true; 
                } else {
                    Serial.println("AXP2101 Begin FAIL");
                }
                if (!axp_192.begin(Wire, AXP192_SLAVE_ADDRESS)) {
                    Serial.println("AXP192 Begin PASS");
                    axp192_found = true; 
                } else {
                    Serial.println("AXP192 Begin FAIL");
                }
            }else{
                Serial.println("axp PMU not found");
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
 * Init the axp_2101 manager chip
 * 
 * axp192 axp_2101 
    DCDC1 0.7-3.5V @ 1200mA max -> OLED  // If you turn this off you'll lose comms to the axp192 because the OLED and the axp192 share the same i2c bus, instead use ssd1306 sleep mode
    DCDC2 -> unused
    DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
    LDO1 30mA -> charges GPS backup battery  // charges the tiny J13 battery by the GPS to axp_2101 the GPS ram (for a couple of days), can not be turned off
    LDO2 200mA -> LORA
    LDO3 200mA -> GPS

 * axp202 axp_2101 
    DCDC1 0.7-3.5V @ 1200mA max -> unused  // If you turn this off you'll lose comms to the axp192 because the OLED and the axp192 share the same i2c bus, instead use ssd1306 sleep mode
    DCDC2 -> OLED
    DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
    LDO1 30mA -> charges GPS backup battery  // charges the tiny J13 battery by the GPS to axp_2101 the GPS ram (for a couple of days), can not be turned off
    LDO2 200mA -> LORA
    LDO3 200mA -> GPS

 */

void axp2101Init() { // Initialization for the power management unit 2101
    if (axp2101_found) {
        
        axp_2101.enableTemperatureMeasure();
        // Enable internal ADC detection
        axp_2101.enableBattDetection();
        axp_2101.enableVbusVoltageMeasure();
        axp_2101.enableBattVoltageMeasure();
        axp_2101.enableSystemVoltageMeasure();

        // Configuration and activation of power gates
        Serial.printf("DCDC1: %s\n", axp_2101.isEnableDC1() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp_2101.isEnableDC2() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp_2101.isEnableALDO2() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp_2101.isEnableALDO3() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp_2101.isEnableDC3() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp_2101.isEnableBLDO1() ? "ENABLE" : "DISABLE");
        Serial.println("----------------------------------------");

        // Voltages configuration and regulators activation
        if (axp_2101.setDC1Voltage(3300) != 0) {  // OLED axp_2101
        Serial.println("Failed to set DC1 voltage");
        }
        axp_2101.setDC2Voltage(3300);  // LORA radio
        //axp_2101.setDC3Voltage(3300);  // GPS main axp_2101
        axp_2101.setBLDO1Voltage(3300);  // External device
        axp_2101.enableDC1();  // Activa DCDC1 para OLED
        axp_2101.enableDC2();  // Activa DCDC2 para LoRa
        axp_2101.enableDC3();  // Activa DCDC3 para GPS
        axp_2101.enableBLDO1();  // Activa BLDO1 para dispositivos externos

        // Verification of power gates after configuration
        Serial.printf("DCDC1: %s\n", axp_2101.isEnableDC1() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp_2101.isEnableDC2() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp_2101.isEnableALDO2() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp_2101.isEnableALDO3() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp_2101.isEnableDC3() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp_2101.isEnableBLDO1() ? "ENABLE" : "DISABLE");

        Serial.printf("DC1  : %s   Voltage:%u mV \n",  axp_2101.isEnableDC1()  ? "+" : "-", axp_2101.getDC1Voltage());

        // Irq configuration
        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
            pmu_irq = true;
        }, FALLING);

        
        /*
        The default setting is CHGLED is automatically controlled by the PMU.
        - XPOWERS_CHG_LED_OFF,
        - XPOWERS_CHG_LED_BLINK_1HZ,
        - XPOWERS_CHG_LED_BLINK_4HZ,
        - XPOWERS_CHG_LED_ON,
        - XPOWERS_CHG_LED_CTRL_CHG,
        * */
        axp_2101.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);


        // Disabling irq del axp to define them
        axp_2101.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

        axp_2101.clearIrqStatus();

        axp_2101.enableIRQ(
        XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
        // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
        );
        
        // Verificar si el cargador está conectado
        if (axp_2101.isCharging()) {
            baChStatus = "Charging";
        }
    } else {
        Serial.println("AXP2101 not found");
    }
}


void axp192Init() { // Initialization for the power management unit 192
    if (axp192_found) {
        if (!axp_192.begin(Wire, AXP192_SLAVE_ADDRESS)) {
            Serial.println("AXP192 Begin PASS");
        } else {
            Serial.println("AXP192 Begin FAIL");
            axpsecure = true; // Gracias a esta variable puedo activar correctamente el axp2101 sin alterar la activación del axp192
        }

        // axp_192.setChgLEDMode(LED_BLINK_4HZ);
        Serial.printf("DCDC1: %s\n", axp_192.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp_192.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp_192.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp_192.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp_192.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp_192.isExtenEnable() ? "ENABLE" : "DISABLE");
        Serial.println("----------------------------------------");

        axp_192.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // LORA radio
        axp_192.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS main axp_2101
        axp_192.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axp_192.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        axp_192.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
        axp_192.setDCDC1Voltage(3300);  // for the OLED axp_2101

        Serial.printf("DCDC1: %s\n", axp_192.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp_192.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp_192.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp_192.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp_192.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp_192.isExtenEnable() ? "ENABLE" : "DISABLE");

        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
            pmu_irq = true;
        }, FALLING);

        axp_192.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
        axp_192.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
        axp_192.clearIRQ();

        if (axp_192.isCharging()) {
            baChStatus = "Charging";
        }
    } else {
        Serial.println("AXP192 not found");
    }
}


// Perform axp_2101 on init that we do on each wake from deep sleep
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

    // Start I2C 
    Wire.begin(I2C_SDA, I2C_SCL);
    scanI2Cdevice();

    // Start power management unit (it only starts the one included, could be improved)
    axp192Init();
    axp2101Init();

    // Buttons & LED
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    #ifdef LED_PIN
        pinMode(LED_PIN, OUTPUT);
    #endif

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

    const char* brokers[] = {
        "srs-copper.duckdns.org", // broker chan
        "broker.emqx.io",         // broker online
        "mqtt-dashboard.com",     // broker online
        "192.168.16.15",          // Broker VPN ambos conectados a la VPN de forma indirecta directa (no funciona)
        "192.168.1.141"           // Conexión local

    };
    const int broker_port = 1883;
    int selection = 0; // Broker selection goes between 0 and 4

    // Broker selection
    const char* selectedBroker = brokers[selection];
    client.setServer(selectedBroker, broker_port);

    // Mostrar el broker seleccionado en la consola serial
    Serial.printf("Broker seleccionado: %s en el puerto %d\n", selectedBroker, broker_port);
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
    // Receiving telemetry
    LoRaReceive();

  if (!client.connected()) { // Try to connect to MQTT server in case it accidentally disconects
    reconnect();
  }
  client.loop();
}
