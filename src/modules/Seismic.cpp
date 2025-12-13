// src/modules/Seismic.cpp
#include "Module.h"
#include <Wire.h>

static class SeismicModule : public Module {
public:
    SeismicModule() : Module("Seismic", 10000) {} // 10s
    virtual void setup() override {
        Wire.begin();
        // Config LIS3DH silencieuse
        write8(0x20, 0x57);
        write8(0x23, 0x88);
    }
    virtual void loop() override {
        float x = read16(0x28) / 16384.0f;
        float y = read16(0x2A) / 16384.0f;
        float z = read16(0x2C) / 16384.0f;
        
        char msg[64];
        snprintf(msg, sizeof(msg), "SEISMIC:%.3f,%.3f,%.3f", x,y,z);
        service.sendText(msg);  // ✅ CLI/Python OK
    }
private:
    int16_t read16(uint8_t reg) {
        Wire.beginTransmission(0x19); Wire.write(reg); Wire.endTransmission(false);
        Wire.requestFrom(0x19, 2);
        return (int16_t)(Wire.read() | (Wire.read() << 8));
    }
    void write8(uint8_t reg, uint8_t v) {
        Wire.beginTransmission(0x19); Wire.write(reg); Wire.write(v); Wire.endTransmission();
    }
} seismicModule;
