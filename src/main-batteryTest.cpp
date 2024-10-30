#include "Arduino.h"
#include "MAX17043.h"

void setup(){
    Serial.begin(9600);
    Wire.begin(4, 0);
    FuelGauge.begin();
}

void loop() {
    float percentage = FuelGauge.percent();
    float voltage = FuelGauge.voltage();

    Serial.print("Battery percentage: ");
    Serial.print(percentage);
    Serial.println("%");

    Serial.print("Battery voltage: ");
    Serial.print(voltage);
    Serial.println("V");

    delay(1000);
}
