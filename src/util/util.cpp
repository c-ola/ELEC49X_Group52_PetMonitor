#include "util/util.h"

void printBinaryByte(byte value) {
    Serial.print("0b");
    for (byte mask = 0x80; mask; mask >>= 1) {
        Serial.print((mask & value) ? '1' : '0');
    }
}
