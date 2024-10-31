#include "Arduino.h"

#define LED_PIN 8  // Pin 3 on ESP32-C3

void setup() {
  pinMode(LED_PIN, OUTPUT);  // Set pin 3 as output
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // Turn on (output high)
  delay(1000);  // Wait 1 second
  digitalWrite(LED_PIN, LOW);   // Turn off (output low)
  delay(1000);  // Wait 1 second
}