/*
 GEIGER COUNTER 1 CODE
 */


const int geigerPin1 = 25; // Pin GPIO 25 in ESP32

volatile unsigned long pulseCount1 = 0;  // Pulse counter

const unsigned long interval1 = 10000;   // Count interval



void SKUSEN0463_setup(void)
{
    Serial.println("SKUSEN0463 setup");
    pinMode(geigerPin1, INPUT);
    attachInterrupt(digitalPinToInterrupt(geigerPin1), countPulse1, RISING);
}  

// Interrupt function
void countPulse1() {
  pulseCount1++;  
}

unsigned long getCP10Sec1() {
  unsigned long counts = pulseCount1;
  pulseCount1 = 0;  // Restart counter
  return counts;
}


