#pragma once
#include "../concurrency/OSThread.h"

class SeismicModule : public concurrency::OSThread {
public:
    SeismicModule();
    virtual int32_t runOnce() override;
private:
    bool initLIS3DH();
    int16_t read16(uint8_t reg);
    void write8(uint8_t reg, uint8_t v);
};
