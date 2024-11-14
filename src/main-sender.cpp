#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <cstdint>
#include <ctime>
#include "Arduino.h"
#include "HardwareSerial.h"
#include "TinyGPS++.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "hal/gpio_types.h"
#include "util/accelerometer.h"
#include "math.h"
#include "LoRa.h"
#include "MAX17043.h"
#include "util/util.h"


/*
 * Pin outs
 *
 *
 */

/*
 * LoRa
 */
#define NSS 5
#define RST 15
#define DI0 4
#define SCK 18
// LoRa-MOSI <=> 23
// LoRa-MISO <=> 19

/*
 * ADXL 345
 */
// INT2
#define ACT_INTERRUPT_PIN GPIO_NUM_34
// SDA <=> 21
// SCL <=> 22
// Wire.begin(21, 22)

/*
 * GPS
    TX <=> 16
    RX <=> 17
    GND <=> Transistor out
    VCC <=> 3v3
 */

/*
 * MAX17043
 */
// Wire.begin(32, 33)

/*
 * Transistor Power Switch
 */
#define ACTIVATE_POWER_PIN GPIO_NUM_25

/* Assign a unique ID to this sensor at the same time */
Accelerometer* accel = new Accelerometer(12345);

float t_now;
float t_last; 
float vx = 0.0f;
float vy = 0.0f;
int state = INACTIVE;
TinyGPSPlus gps;
RTC_DATA_ATTR int boot_count = 0;


#define GPS_BAUD 9600

#define DO_DEEP_SLEEP

void setup() {
    Serial.begin(115200);
    if (boot_count != 0) {
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
        switch (wakeup_reason) {
            case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
            case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
            case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
            case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
            case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
            default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
        }
    }
    Serial.print("Boot Count: "); Serial.println(boot_count);
    boot_count++;

    // RX2 => 16
    // TX2 => 17
    Serial2.begin(GPS_BAUD, SERIAL_8N1, RX2, TX2);

    Serial.println("Setting up Accelerometer");
    Wire.begin(21, 22);
    accel->setup();

    Serial.println("Setup Pins");
    pinMode(2, OUTPUT);
    pinMode(ACTIVATE_POWER_PIN, OUTPUT);
    digitalWrite(ACTIVATE_POWER_PIN, HIGH);
    pinMode(ACT_INTERRUPT_PIN, INPUT);

    Serial.println("Setup LoRa");

    LoRa.setPins(NSS, RST, DI0);
    while (!LoRa.begin(915E6)) {
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF1);
    Serial.println("LoRa Initializing Successful!");

    esp_sleep_enable_ext0_wakeup(ACT_INTERRUPT_PIN, 1);
    rtc_gpio_pullup_dis(ACT_INTERRUPT_PIN);
    rtc_gpio_pulldown_en(ACT_INTERRUPT_PIN);

    delay(100);

    // Fuel Gauge
    Wire1.begin(32, 33);
    FuelGauge.begin(&Wire1);
    if (FuelGauge.isSleeping()) {
        FuelGauge.wake();
        FuelGauge.quickstart();
    } else {
        FuelGauge.reset();
    }


    state = ACTIVE;
    t_now = micros();
    Serial.println("Completed Setup");
}


#ifndef DO_DEEP_SLEEP
void check_active() {
    int act_int = digitalRead(ACT_INTERRUPT_PIN);
    if (act_int && state == INACTIVE) {
        state = ACTIVE;
        Serial.println("Active");
        digitalWrite(LED_BUILTIN, HIGH);
        vx = 0.0f;
        vy = 0.0f;
    } 
}
#endif

bool check_inactive() {
    uint8_t val = accel->readRegister(ADXL345_REG_INT_SOURCE);
    int inact = (val & 0b00001000) == 0b00001000;
    return inact;
}

void do_active() {
    t_last = t_now;
    t_now = micros();
    float delta_time = (t_now - t_last)/1000000;
    if (check_inactive()) {
        Serial.println("Inactive");
        state = INACTIVE;
        digitalWrite(2, LOW);
    } else {
        sensors_event_t event; 
        accel->getEvent(&event);
        vx += event.acceleration.x * delta_time;
        vy += event.acceleration.y * delta_time;
        Serial.print("vx: "); Serial.print(vx); Serial.print(", vy: "); Serial.print(vy); Serial.println();
    }

    float percentage = FuelGauge.percent();
    float voltage = FuelGauge.voltage();

    Serial.print("Battery percentage: ");
    Serial.print(percentage);
    Serial.println("%");

    Serial.print("Battery voltage: ");
    Serial.print(voltage);
    Serial.println("mV");


    /* Get GPS data */
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("Location lat, lng"); Serial.print(gps.location.lat()); Serial.print(","); Serial.println(gps.location.lng());

    Packet pkt = {
        gps.satellites.value(),
        gps.location.lat(),
        gps.location.lng(),
        percentage,
        voltage
    };

    /* Send GPS data */
    if (LoRa.beginPacket()) {
        unsigned int written = LoRa.write((uint8_t*)&pkt, sizeof(Packet));
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

    delay(500);
}


void loop() {
#ifndef DO_DEEP_SLEEP
    check_active();
#endif


    if (state == ACTIVE) {
        digitalWrite(2, HIGH);
        vx = 0.0f;
        vy = 0.0f;
        do_active();
    }
    if (state == INACTIVE) {
        Serial.println("Entering Deep Sleep");
        Serial.flush();
        digitalWrite(ACTIVATE_POWER_PIN, LOW);
        FuelGauge.sleep();
        esp_deep_sleep_start();
    }
}
