#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include "util/util.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NSS 4
#define RST 5
#define DI0 2
#define SCK 18

String LoRaData;

void setup() {
    Serial.begin(115200);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("here");
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    LoRa.setPins(NSS, RST, DI0);

    while (!LoRa.begin(433E6)) {
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF1);
    Serial.println("LoRa Initializing Successful (Receiver)!");

    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

}

uint8_t buf[255];

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.println("Received packet: ");
        while (LoRa.available()) {
            size_t bytes_read = LoRa.readBytes(buf, sizeof(Packet));
        }
        Packet* data = (Packet*)&buf;
        
        Serial.println(data->numSats);
        Serial.println(data->lat);
        Serial.println(data->lng);
        Serial.println(data->percentage);

        int percentage = (int) data->percentage;
        
        display.clearDisplay();
        display.setCursor(10,10);
        display.print(percentage);
        display.display();
        delay(50);
    }
}
