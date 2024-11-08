#pragma once
#include "Arduino.h"

struct Packet {
    uint32_t numSats;
    double lng;
    double lat;
    float percentage;
    float voltage;
};

void printBinaryByte(byte value);
