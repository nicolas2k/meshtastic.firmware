#pragma once
#include "TelemetrySensor.h"
#include <Wire.h>

class LIS3DHSensor : public TelemetrySensor {
public:
    LIS3DHSensor();
    virtual int32_t runOnce() override;  // ✅ Méthode CORRECTE
private:
    uint8_t addr = 0x19;
    bool initSensor();
    int16_t read16(uint8_t reg);
    void write8(uint8_t reg, uint8_t v);
};
