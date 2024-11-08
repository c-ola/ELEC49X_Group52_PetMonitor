#include "util/accelerometer.h"
#include "Adafruit_ADXL345_U.h"

Accelerometer::Accelerometer() : Adafruit_ADXL345_Unified(12345) {

}


Accelerometer::Accelerometer(uint32_t SD) : Adafruit_ADXL345_Unified(SD) {
;
}

void Accelerometer::setup() {

    if(!this->begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while(1);
    }

    //this->displaySensorDetails();
    //this->displayDataRate();
    //this->displayRange();
    
    this->setRange(ADXL345_RANGE_16_G);
    // this->setRange(ADXL345_RANGE_8_G);
    // this->setRange(ADXL345_RANGE_4_G);
    // this->setRange(ADXL345_RANGE_2_G);

    
    this->writeRegister(ADXL345_REG_INT_ENABLE, 0b00011010);
    //this->writeRegister(ADXL345_REG_INT_ENABLE, ADXL345_INT_EN_INACT);
    this->writeRegister(ADXL345_REG_INT_MAP, 0b00010000);
    
    this->writeRegister(ADXL345_REG_THRESH_ACT, 5);
    this->writeRegister(ADXL345_REG_THRESH_INACT, 20);
    this->writeRegister(ADXL345_REG_TIME_INACT, 10);
    
    this->writeRegister(ADXL345_REG_ACT_INACT_CTL, 0b01100110);

}

void Accelerometer::displaySensorDetails() {
    sensor_t sensor;
    this->getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}


void Accelerometer::displayDataRate() {
    Serial.print("Data Rate:    "); 

    switch(this->getDataRate())
    {
        case ADXL345_DATARATE_3200_HZ:
            Serial.print  ("3200 "); 
            break;
        case ADXL345_DATARATE_1600_HZ:
            Serial.print  ("1600 "); 
            break;
        case ADXL345_DATARATE_800_HZ:
            Serial.print  ("800 "); 
            break;
        case ADXL345_DATARATE_400_HZ:
            Serial.print  ("400 "); 
            break;
        case ADXL345_DATARATE_200_HZ:
            Serial.print  ("200 "); 
            break;
        case ADXL345_DATARATE_100_HZ:
            Serial.print  ("100 "); 
            break;
        case ADXL345_DATARATE_50_HZ:
            Serial.print  ("50 "); 
            break;
        case ADXL345_DATARATE_25_HZ:
            Serial.print  ("25 "); 
            break;
        case ADXL345_DATARATE_12_5_HZ:
            Serial.print  ("12.5 "); 
            break;
        case ADXL345_DATARATE_6_25HZ:
            Serial.print  ("6.25 "); 
            break;
        case ADXL345_DATARATE_3_13_HZ:
            Serial.print  ("3.13 "); 
            break;
        case ADXL345_DATARATE_1_56_HZ:
            Serial.print  ("1.56 "); 
            break;
        case ADXL345_DATARATE_0_78_HZ:
            Serial.print  ("0.78 "); 
            break;
        case ADXL345_DATARATE_0_39_HZ:
            Serial.print  ("0.39 "); 
            break;
        case ADXL345_DATARATE_0_20_HZ:
            Serial.print  ("0.20 "); 
            break;
        case ADXL345_DATARATE_0_10_HZ:
            Serial.print  ("0.10 "); 
            break;
        default:
            Serial.print  ("???? "); 
            break;
    }  
    Serial.println(" Hz");  
}

void Accelerometer::displayRange() {
    Serial.print  ("Range:         +/- "); 

    switch(this->getRange())
    {
        case ADXL345_RANGE_16_G:
            Serial.print  ("16 "); 
            break;
        case ADXL345_RANGE_8_G:
            Serial.print  ("8 "); 
            break;
        case ADXL345_RANGE_4_G:
            Serial.print  ("4 "); 
            break;
        case ADXL345_RANGE_2_G:
            Serial.print  ("2 "); 
            break;
        default:
            Serial.print  ("?? "); 
            break;
    }  
    Serial.println(" g");  
}
