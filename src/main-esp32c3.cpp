//#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>

// #define NSS 3
// #define RST 4
// #define DI0 2
// #define SCK 8
// #define MISO 9
// #define MOSI 10

#define NSS   2   // Chip Select
#define RST   3   // Reset
#define DI0   4   // Interrupt pin
#define SCK   8   // Serial Clock
#define MISO  9   // Master In Slave Out
#define MOSI  10

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
