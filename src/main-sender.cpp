#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ctime>
#include "Arduino.h"
#include "HardwareSerial.h"
#include "TinyGPS++.h"
#include "util/accelerometer.h"
#include "util/util.h"
#include "math.h"
#include "LoRa.h"

/* Assign a unique ID to this sensor at the same time */
Accelerometer* accel = new Accelerometer(12345);

float t_now;
float t_last; 
float vx = 0.0f;
float vy = 0.0f;
int state = INACTIVE;
TinyGPSPlus gps;

#define NSS 5
#define RST 15
#define DI0 4
#define SCK 18

#define LED_PIN 2
#define ACT_INTERRUPT_PIN 34

#define GPS_BAUD 9600

void setup() {
    Serial.begin(115200);
    // RX2 => 16
    // TX2 => 17
    Serial2.begin(GPS_BAUD, SERIAL_8N1, RX2, TX2);
    
    Serial.println("Setting up Accelerometer");
    accel->setup();

    Serial.println("Setup Pins");
    pinMode(LED_PIN, OUTPUT);
    pinMode(ACT_INTERRUPT_PIN, INPUT);
    
    Serial.println("Setup LoRa");

    LoRa.setPins(NSS, RST, DI0);
    while (!LoRa.begin(433E6)) {
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF1);
    Serial.println("LoRa Initializing Successful!");


    t_now = micros();
    Serial.println("Completed Setup");
}


void loop() {
    /* Get a new sensor event */ 
    byte val;
    //val = accel.readRegister(ADXL345_REG_INT_SOURCE);
    //Serial.print("INT_SOURCE: "); printBinaryByte(val); Serial.println();

    int act_int = digitalRead(ACT_INTERRUPT_PIN);
    if (act_int && state == INACTIVE) {
        state = ACTIVE;
        Serial.println("Active");
        digitalWrite(LED_PIN, HIGH);
        vx = 0.0f;
        vy = 0.0f;
    } 
    if (state == ACTIVE) {
        t_last = t_now;
        t_now = micros();
        float delta_time = (t_now - t_last)/1000000;
        val = accel->readRegister(ADXL345_REG_INT_SOURCE);
        //Serial.print("INT_SOURCE: "); printBinaryByte(val); Serial.println();
        
        /* Figure out if ACTIVE or INACTIVE */
        int inact = val & 0b00001000;
        if (inact) {
            Serial.println("Inactive");
            state = INACTIVE;
            digitalWrite(LED_PIN, LOW);
        } else {
            sensors_event_t event; 
            accel->getEvent(&event);
            vx += event.acceleration.x * delta_time;
            vy += event.acceleration.y * delta_time;
            Serial.print("vx: "); Serial.print(vx); Serial.print(", vy: "); Serial.print(vy); Serial.println();
        }
        
        /* Get GPS data */
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
        Serial.print("Location lat, lng"); Serial.print(gps.location.lat()); Serial.print(gps.location.lng());
        

        /* Send GPS data */
        if (LoRa.beginPacket()) {
            uint8_t buf[255] = "hello\0";

            unsigned int written = LoRa.write(buf, 255);
            Serial.print("Written: "); Serial.println(written);
            if (LoRa.endPacket(false)) {
                Serial.println("Sent packet");
            } else {
                Serial.println("Failed to send packet");
            }
        }

        while (Serial2.available()) {
            gps.encode(Serial2.read());
        }

    }
}
