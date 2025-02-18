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
#include "axp20x.h"
#include "Arduino.h"
#include "LittleFS.h"
#include "logger.h"
#include "FS.h"
#include "LogEntryTests.h"

#define XPOWERS_CHIP_AXP2101 // Macro that defines the Power Management Unit we are going to use in the new ESP32 modules

#include "XPowersLib.h" // The library needed for previously mentionned library


#define FORMAT_LITTLEFS_IF_FAILED true // Allows to format archyve system and reestart it again if needed so that the mission is able to continue (memory data would be erased as a side effect)

void WebFS(void); // Web server declaration
void gps_time(char *, uint8_t);

// char t_buf[16];



AXP20X_Class axp_192; // PMU ancient ESP32
XPowersPMU axp_2101; // Pmu brand new ESP32
bool pmu_irq = false;


String baChStatus = "No charging";

bool ssd1306_found = false;
bool axp192_found = false;
bool axp2101_found = false;
bool bme280_found = false; //Para ver si el bme está soldado
bool DS18B20_found = false; //Para ver si el sensor de temperatura está soldado
bool SKUSEN0463_found = false; //Para ver si el geiger está soldado

bool packetSent, packetQueued;
int counter = 0; //Contador de mensajes

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause;  // the reason we booted this time

/* Se ha pasado al datastruct.h
typedef struct {
    byte TX_ID = 0x55;
    byte RX_ID = 0xAA;
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
    //unsigned int CP10Sec_Libellium;
} Data;

Data data; // Data structure deffinition relocated in logger.h
*/

char log_entry[LINE_SIZE];

void readFile(const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = LittleFS.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
    //if(file.available()){    
        Serial.write(file.read());
    }
    file.close();
}

// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------

/**
 * If we have a valid position send it to the server.
 */
void LoRaSend() {
    char buffer[64];
    // We also wait for altitude being not exactly zero, because the GPS chip generates a bogus 0 alt report when first powered on
    if (0 < gps_hdop() && gps_hdop() < 50 && gps_latitude() != 0 && gps_longitude() != 0 && gps_altitude() != 0) {

        snprintf(buffer, sizeof(buffer), "Latitude: %10.6f\n", gps_latitude());
        screen_print(buffer);
        snprintf(buffer, sizeof(buffer), "Longitude: %10.6f\n", gps_longitude());
        screen_print(buffer);
        snprintf(buffer, sizeof(buffer), "Altitude: %10.6f\n", gps_altitude());
        screen_print(buffer);
        snprintf(buffer, sizeof(buffer), "Error: %4.2fm\n", gps_hdop());
        screen_print(buffer);
    }
    
    /* Chan: Enviar paquete LoRa */
    /*
    LoRa.beginPacket();
    LoRa.write((byte *)&data, sizeof(Data));
    LoRa.endPacket();
    */
    LoRa.beginPacket();
    if (LoRa.write((byte *)&data, sizeof(Data)) == sizeof(Data)) {
        LoRa.endPacket();
        Serial.println("[INFO] Paquete LoRa enviado correctamente.");
    } else {
        Serial.println("[ERROR] No se pudo escribir el paquete LoRa.");
    }
    Serial.print ("Size of data: ");
    Serial.println(sizeof(Data));
    Serial.print ("Number of sats: ");
    Serial.println(gps_sats());
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
        axp_192.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // LORA radio
        axp_192.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS main power
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
                Serial.println("axp PMU found");
                
                if (axp_2101.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
                    Serial.println("AXP2101 Begin PASS");
                    axp2101_found = true; // New PMU
                } else {
                    Serial.println("AXP2101 Begin FAIL");
                }
                if (!axp_192.begin(Wire, AXP192_SLAVE_ADDRESS)) {
                    Serial.println("AXP192 Begin PASS");
                    axp192_found = true; // Ancient PMU
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
        if (!axp_192.begin(Wire, AXP192_SLAVE_ADDRESS)) {
            Serial.println("AXP192 Begin PASS");
        } else {
            Serial.println("AXP192 Begin FAIL");
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
        axp_192.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS main power
        axp_192.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axp_192.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        axp_192.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
        axp_192.setDCDC1Voltage(3300);  // for the OLED power

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

void axp2101Init() {
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
        if (axp_2101.setDC1Voltage(3300)!=0) { // OLED axp_2101
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
        

        // Charging flag
        if (axp_2101.isCharging()) {
            baChStatus = "Charging";
        }
    } else {
        Serial.println("AXP2101 not found");
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
    axp2101Init();

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
    //Init BME 280
    bme280_setup(); 
    //Init external temperature
    //DFRobot_max31855_setup(); // Previous probe model from Chan
    DS18B20_setup(); 
    // Init SKUSEN0463 geiger counter
    Gravity_setup();
    //Libellium_setup();
    
    // Init LoRa
    lora_setup();


    if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED))
    {    
        Serial.println("LitlleFS Mount Failed");
        return;
    }
    //readFile(LOGFILE);

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
}

void loop() {
    gps_loop(); 
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
            screen_print("IO38 is pressed\n");
            WebFS();
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

    data.latitude        = gps_latitude();
    data.longitude       = gps_longitude();
    data.altitude        = gps_altitude();
    data.hdop            = gps_hdop();
    if (bme280_found){
        data.temperature     = bme280_temperature();
        data.pressure        = bme280_pressure();
        data.humidity        = bme280_humidity();
        data.baro_altitude   = bme280_baro_altitude();
    }else{
        data.temperature     = bme280_temperature_dummy();
        data.pressure        = bme280_pressure_dummy();
        data.humidity        = bme280_humidity_dummy();
        data.baro_altitude   = bme280_baro_altitude_dummy();
    }
    //data.ext_temperature_ours = DFRobot_max31855_temperature(); //Sonda de Chan
    data.ext_temperature_ours = DS18B20_tempC(); // Sonda nuestra
    //data.ext_temperature_ours = DS18B20_tempC_dummy();

    data.CP10Sec_Gravity         = getCP10Sec_Gravity();
    //data.CP10Sec_Libellium       = getCP10Sec_Libellium();

    //data.CP10Sec_Gravity         = getCP10Sec_dummy();
    //data.CP10Sec_Libellium       = getCP10Sec_dummy();

    // Send every 10000 millis
    LoRaSend();
    data.seq++;

    gps_time(t_buf, sizeof(t_buf));
    //LogEntryOriginal(log_entry, data, t_buf);
    //LogEntryWithTruncateCheck(log_entry, data, t_buf); // V1
    LogEntryWithSpaces(log_entry, data, t_buf); // V2
    //LogEntryWithInterruptProtection(log_entry, data, t_buf); // V3

    //logger(log_entry); // Max. 22272 records de 64 bytes
    Serial.print ("Size of log: ");
    Serial.println(sizeof(log_entry));
    Serial.println("Tiempo, Pressure, T_ext,T_int, Humidity, altitude (m), longitude, latitude, Geig_Grav");
    Serial.println(log_entry);
    Serial.println(baChStatus);
    Serial.print("  Batt Voltage = "); 
    Serial.println((float)axp_2101.getBattVoltage()/1000.0);  
    Serial.println(axp_2101.getBatteryPercent());  


    delay(10000);
}
