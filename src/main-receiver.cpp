#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include "util/util.h"

#define NSS 4
#define RST 5
#define DI0 2
#define SCK 18

String LoRaData;

void setup() {
    Serial.begin(115200);
    LoRa.setPins(NSS, RST, DI0);

    while (!LoRa.begin(433E6)) {
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF1);
    Serial.println("LoRa Initializing Successful (Receiver)!");

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
    }
}
