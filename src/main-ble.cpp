#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>

// Define service and characteristic UUIDs
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define DEVICE_NAME "ESP32_BLE"

// Global variables
float latitude = 37.7749; // Example latitude
float longitude = -122.4194; // Example longitude
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
BLEServer *pServer;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("Device connected");
    }
    
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("Device disconnected");
        // Restart advertising when disconnected
        pServer->startAdvertising();
        Serial.println("Started advertising again");
    }
};

void setup() {
    Serial.begin(115200);
    
    // Initialize BLE device
    BLEDevice::init(DEVICE_NAME);
    
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Create BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    // Start the service
    pService->start();
    
    // Configure advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMaxPreferred(0x12);
    
    // Set advertising parameters (optional but recommended)
    pAdvertising->setMinInterval(0x20); // 0x20 * 0.625ms = 20ms
    pAdvertising->setMaxInterval(0x40); // 0x40 * 0.625ms = 40ms
    
    BLEDevice::startAdvertising();
    Serial.println("BLE Device is advertising!");
}

void loop() {
    if (deviceConnected) {
        String message = String(latitude, 6) + "," + String(longitude, 6);
        pCharacteristic->setValue(message.c_str());
        pCharacteristic->notify();
        delay(1000);
    }
}