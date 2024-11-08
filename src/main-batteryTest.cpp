#include "Arduino.h"
#include "MAX17043.h"

void setup(){
    Serial.begin(115200);
    Wire.begin(21, 22);
    FuelGauge.begin();
    FuelGauge.quickstart();
}

void loop() {
    float percentage = FuelGauge.percent();
    float voltage = FuelGauge.voltage();
    
    Serial.print("Battery percentage: ");
    Serial.print(percentage);
    Serial.println("%");

    Serial.print("Battery voltage: ");
    Serial.print(voltage);
    Serial.println("mV");

    delay(1000);
}
