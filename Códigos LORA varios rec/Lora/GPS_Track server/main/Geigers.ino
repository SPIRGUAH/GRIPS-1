/*
 GEIGER COUNTER CODE
 */

const int geigerPin_Gravity = 25;  // Pin GPIO 25 in ESP32
//const int geigerPin_Libellium = 35; // Pin GPIO 32 in ESP32

volatile unsigned long pulseCount_Gravity = 0;  // Pulse counter for Gravity Geiger
//volatile unsigned long pulseCount_Libellium = 0;  // Pulse counter for Libellium Geiger

const unsigned long interval = 10000;   // Count interval

void Gravity_setup(void) {
    Serial.println("Gravity setup");
    pinMode(geigerPin_Gravity, INPUT);
    attachInterrupt(digitalPinToInterrupt(geigerPin_Gravity), countPulse_Gravity, RISING);
}  

/*
void Libellium_setup(void) {
    Serial.println("Libellium setup");
    pinMode(geigerPin_Libellium, INPUT);
    attachInterrupt(digitalPinToInterrupt(geigerPin_Libellium), countPulse_Libellium, RISING);
} 
*/

// Interrupt function
void countPulse_Gravity() {
  detachInterrupt(digitalPinToInterrupt(geigerPin_Gravity));
  pulseCount_Gravity++;  
  attachInterrupt(digitalPinToInterrupt(geigerPin_Gravity), countPulse_Gravity, RISING);
}

/*
void countPulse_Libellium() {
  detachInterrupt(digitalPinToInterrupt(geigerPin_Libellium));
  pulseCount_Libellium++;
  attachInterrupt(digitalPinToInterrupt(geigerPin_Libellium), countPulse_Libellium, RISING);  
}
*/

unsigned long getCP10Sec_Gravity() {
  unsigned long counts = pulseCount_Gravity;
  pulseCount_Gravity = 0;  // Restart counter
  return counts;
}

/*
unsigned long getCP10Sec_Libellium() {
  unsigned long counts = pulseCount_Libellium;
  pulseCount_Libellium = 0;  // Restart counter
  return counts;
}
*/