#include "LD2450.h"
#include <math.h>

LD2450::LD2450() : _serial(nullptr) {
    for (int i = 0; i < 3; i++) {
        _targets[i] = {false, 0, 0, 0, 0, 0};
    }
}

void LD2450::begin(HardwareSerial& serial, bool skipInit) {
    _serial = &serial;

    if (!skipInit) {
        _serial->begin(256000, SERIAL_8N1, 16, 17);
    }
}

int16_t LD2450::bytesToInt16(uint8_t low, uint8_t high) {
    return (int16_t)(low | (high << 8));
}

void LD2450::parseFrame(const uint8_t* frame, size_t frameLen) {
    if (frameLen < 30) return;

    int dataIndex = 4;

    for (int i = 0; i < 3; i++) {

        if (dataIndex + 8 <= frameLen - 2) {

            int16_t x = bytesToInt16(
                frame[dataIndex],
                frame[dataIndex + 1]
            );

            int16_t y = bytesToInt16(
                frame[dataIndex + 2],
                frame[dataIndex + 3]
            );

            int16_t speed = bytesToInt16(
                frame[dataIndex + 4],
                frame[dataIndex + 5]
            );

            uint16_t distance =
                (uint16_t)sqrt((long)x * x + (long)y * y);

            _targets[i].valid = (x != 0 || y != 0);
            _targets[i].id = i + 1;
            _targets[i].x = x;
            _targets[i].y = y;
            _targets[i].speed = speed;
            _targets[i].distance = distance;

            dataIndex += 8;
        }
    }
}

int LD2450::read() {

    if (!_serial) return 0;

    for (int i = 0; i < 3; i++) {
        _targets[i].valid = false;
    }

    if (_serial->available() < 30) {
        return 0;
    }

    uint8_t buffer[256];
    size_t bytesRead = 0;

    while (_serial->available() > 0 &&
           bytesRead < sizeof(buffer)) {

        buffer[bytesRead++] = _serial->read();
    }

    for (size_t i = 0; i + 29 < bytesRead; i++) {

        if (buffer[i] == 0xAA &&
            buffer[i + 1] == 0xFF &&
            buffer[i + 2] == 0x03 &&
            buffer[i + 3] == 0x00 &&
            buffer[i + 28] == 0x55 &&
            buffer[i + 29] == 0xCC) {

            parseFrame(&buffer[i], 30);

            int count = 0;

            for (int j = 0; j < 3; j++) {
                if (_targets[j].valid) {
                    count++;
                }
            }

            return count;
        }
    }

    return 0;
}

LD2450::RadarTarget LD2450::getTarget(int index) {

    if (index < 0 || index > 2) {
        return {false, 0, 0, 0, 0, 0};
    }

    return _targets[index];
}