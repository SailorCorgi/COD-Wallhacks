#ifndef LD2450_H
#define LD2450_H

#include <Arduino.h>
#include <HardwareSerial.h>

class LD2450 {
public:
    struct RadarTarget {
        bool valid;
        uint16_t id;
        int16_t x;       // mm
        int16_t y;       // mm
        int16_t distance; // mm
        int16_t speed;   // cm/s
    };

    LD2450();
    
    // Initialize with HardwareSerial
    // skipInit: if true, assumes Serial is already initialized; if false, this will call Serial.begin()
    void begin(HardwareSerial& serial, bool skipInit = false);
    
    // Read data from sensor
    // Returns number of targets found (0-3)
    int read();
    
    // Get target by index (0-2)
    RadarTarget getTarget(int index);
    
private:
    HardwareSerial* _serial;
    RadarTarget _targets[3];
    
    void parseFrame(const uint8_t* frame, size_t frameLen);
    int16_t bytesToInt16(uint8_t low, uint8_t high);
};

#endif
