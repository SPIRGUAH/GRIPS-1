/*
 GEIGER COUNTER 2 CODE
 */


const int geigerPin2 = 32; // Pin GPIO 25 in ESP32

volatile unsigned long pulseCount2 = 0;  // Pulse counter

const unsigned long interval = 10000;   // Count interval



void libelium_setup(void)
{
    Serial.println("libelium setup");
    pinMode(geigerPin2, INPUT);
    attachInterrupt(digitalPinToInterrupt(geigerPin2), countPulse2, RISING);
}  

// Interrupt function
void countPulse2() {
  pulseCount2++;  
}

unsigned long getCP10Sec2() {
  unsigned long counts = pulseCount2;
  pulseCount2 = 0;  // Restart counter
  return counts;
}