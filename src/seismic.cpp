#include "configuration.h"
#include <Wire.h>

static int32_t seismic_run() {
    static bool init = false;
    if (!init) {
        Wire.begin();
        Wire.beginTransmission(0x19);
        Wire.write(0x0F);
        if (Wire.endTransmission() == 0) {
            Wire.requestFrom(0x19, 1);
            if (Wire.read() == 0x33) {
                Wire.beginTransmission(0x19);
                Wire.write(0x20); Wire.write(0x57); Wire.endTransmission();
                Wire.beginTransmission(0x19);
                Wire.write(0x23); Wire.write(0x88); Wire.endTransmission();
                init = true;
                LOG_INFO("LIS3DH seismic ready\n");
            }
        }
        return 5000;
    }
    
    int16_t x = 0, y = 0, z = 0;
    Wire.beginTransmission(0x19); Wire.write(0x28); Wire.endTransmission(false);
    Wire.requestFrom(0x19, 2); x = (int16_t)(Wire.read() | (Wire.read() << 8));
    Wire.beginTransmission(0x19); Wire.write(0x2A); Wire.endTransmission(false);
    Wire.requestFrom(0x19, 2); y = (int16_t)(Wire.read() | (Wire.read() << 8));
    Wire.beginTransmission(0x19); Wire.write(0x2C); Wire.endTransmission(false);
    Wire.requestFrom(0x19, 2); z = (int16_t)(Wire.read() | (Wire.read() << 8));
    
    char msg[64];
    snprintf(msg, sizeof(msg), "SEISMIC:%.3f,%.3f,%.3f", x/16384.0f, y/16384.0f, z/16384.0f);
    service.sendText(msg);
    
    LOG_INFO("%s\n", msg);
    return 10000; // 10s
}

void setupSeismic() {
    scheduler.scheduleTask(0, 10000, seismic_run);
}
