#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>

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
  Serial.println("LoRa Initializing Successful!");

}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
    }
    Serial.println(LoRaData);
  }
}
