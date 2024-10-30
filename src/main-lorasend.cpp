#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <cstdint>

#define NSS 5
#define RST 15
#define DI0 4
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
    Serial.println("LoRa Initializing Successful!");
}

void loop() {
    while(!LoRa.beginPacket());
    Serial.println("Preparing packet");
    uint8_t buffer[255] = "hello daniell\0"; 
    unsigned int written = LoRa.write(buffer, 255);
    Serial.print("Written: "); Serial.println(written);
    if (LoRa.endPacket(false)) {
        Serial.println("Sent packet");
    } else {
        Serial.println("Failed to send packet");
    }

}
