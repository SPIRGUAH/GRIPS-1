/*
 GEIGER COUNTER CODE
 */


const int geigerPin = 25; // Pin GPIO 21 in ESP32

volatile unsigned long pulseCount = 0;  // Pulse counter

const unsigned long interval = 10000;   // Count interval



void SKUSEN0463_setup(void)
{
    Serial.println("SKUSEN0463 setup");
    pinMode(geigerPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(geigerPin), countPulse, RISING);
}  

// Interrupt function
void countPulse() {
  pulseCount++;  
}

unsigned long getCP10Sec() {
  unsigned long counts = pulseCount;
  pulseCount = 0;  // Restart counter
  return counts;
}


