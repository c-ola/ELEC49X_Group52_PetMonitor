#pragma once

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define ADXL345_INT_EN_ACT (0b00010000)
#define ADXL345_INT_EN_INACT (0b00001000)
#define ACTIVE 1
#define INACTIVE 0


class Accelerometer : public Adafruit_ADXL345_Unified {
public:
    Accelerometer();
    Accelerometer(uint32_t SD = -1);
    ~Accelerometer() = default;
    void setup();
    
    void displaySensorDetails();
    void displayDataRate();
    void displayRange();
};


