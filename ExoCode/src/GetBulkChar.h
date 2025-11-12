#ifndef GetBulkChar_h
#define GetBulkChar_h

#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
//#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)


void readSingleMessageBlocking();
#endif
#endif