#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ctime>
#include "Arduino.h"

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void displaySensorDetails(void)
{
    sensor_t sensor;
    accel.getSensor(&sensor);
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

void displayDataRate(void)
{
    Serial.print  ("Data Rate:    "); 

    switch(accel.getDataRate())
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

void displayRange(void)
{
    Serial.print  ("Range:         +/- "); 

    switch(accel.getRange())
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

#define ADXL345_INT_EN_ACT (0b00010000)
#define ADXL345_INT_EN_INACT (0b00001000)
#define LED_PIN 2

float t_now;
float t_last; 

void setup(void) 
{
    Serial.begin(9600);
    Serial.println("Accelerometer Test"); Serial.println("");
    /* Initialise the sensor */
    if(!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while(1);
    }

    /* Set the range to whatever is appropriate for your project */
    accel.setRange(ADXL345_RANGE_16_G);
    // accel.setRange(ADXL345_RANGE_8_G);
    // accel.setRange(ADXL345_RANGE_4_G);
    // accel.setRange(ADXL345_RANGE_2_G);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Display additional settings (outside the scope of sensor_t) */
    displayDataRate();
    displayRange();
    
    accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b00011010);
    //accel.writeRegister(ADXL345_REG_INT_ENABLE, ADXL345_INT_EN_INACT);
    accel.writeRegister(ADXL345_REG_INT_MAP, 0b00010000);
    
    accel.writeRegister(ADXL345_REG_THRESH_ACT, 5);
    accel.writeRegister(ADXL345_REG_THRESH_INACT, 20);
    accel.writeRegister(ADXL345_REG_TIME_INACT, 0x02);
    
    accel.writeRegister(ADXL345_REG_ACT_INACT_CTL, 0b01100110);

    Serial.println("");
    pinMode(LED_PIN, OUTPUT);
    pinMode(16, OUTPUT);
    pinMode(4, INPUT);
    
    t_now = micros();
}

void printBinaryByte(byte value) {
    Serial.print("0b");
    for (byte mask = 0x80; mask; mask >>= 1) {
        Serial.print((mask & value) ? '1' : '0');
    }
}

#define ACTIVE 1
#define INACTIVE 0
int state = INACTIVE;

#include "math.h"
float vx = 0.0f;
float vy = 0.0f;

void loop(void) 
{
    /* Get a new sensor event */ 
    byte val;
    //val = accel.readRegister(ADXL345_REG_INT_SOURCE);
    //Serial.print("INT_SOURCE: "); printBinaryByte(val); Serial.println();

    int pin4 = digitalRead(4);
    if (pin4 && state == INACTIVE) {
        state = ACTIVE;
        Serial.println("Active");
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(16, HIGH);
        vx = 0.0f;
        vy = 0.0f;
    } 
    if (state == ACTIVE) {
        t_last = t_now;
        t_now = micros();
        float delta_time = (t_now - t_last)/1000000;
        val = accel.readRegister(ADXL345_REG_INT_SOURCE);
        //Serial.print("INT_SOURCE: "); printBinaryByte(val); Serial.println();
        int inact = val & 0b00001000;
        if (inact) {
            Serial.println("Inactive");
            state = INACTIVE;
            digitalWrite(LED_PIN, LOW);
            digitalWrite(16, LOW);
        } else {
            sensors_event_t event; 
            accel.getEvent(&event);
            vx += event.acceleration.x * delta_time;
            vy += event.acceleration.y * delta_time;
            Serial.print("vx: "); Serial.print(vx); Serial.print(", vy: "); Serial.print(vy); Serial.println();
        }

    }
    /* Display the results (acceleration is measured in m/s^2) */
    //Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    //Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    //Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}
