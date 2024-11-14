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
    Wire.begin(19, 23);

    while (!LoRa.begin(915E6)) {
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF1);
    Serial.println("LoRa Initializing Successful!");
}

int counter = 0;
void loop() {
  // wait until the radio is ready to send a packet
  while (LoRa.beginPacket() == 0) {
    Serial.print("waiting for radio ... ");
    delay(100);
  }
  Serial.print("Sending packet non-blocking: ");
  Serial.println(counter);

  // send in async / non-blocking mode
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket(); // true = async / non-blocking mode

  counter++;
}
