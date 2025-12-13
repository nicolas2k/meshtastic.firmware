#include "LIS3DHSensor.h"
#include "../../../mesh/generated/meshtastic/telemetry.pb.h"
#include "configuration.h"

LIS3DHSensor::LIS3DHSensor() : TelemetrySensor("LIS3DH", meshtastic_TelemetrySensorType_SENSOR_UNSET) {}

bool LIS3DHSensor::init() {
    Wire.begin();
    Wire.beginTransmission(addr);
    Wire.write(0x0F);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom(addr, 1);
    if (Wire.read() != 0x33) return false;
    
    write8(0x20, 0x57); // 100Hz XYZ
    write8(0x23, 0x88); // ±16g
    LOG_INFO("LIS3DH ready\n");
    return true;
}

meshtastic_Telemetry *LIS3DHSensor::getReading(meshtastic_Telemetry *m) {
    m->variant.device.accel_x_g = read16(0x28) / 16384.0f;  // ✅ device.accel_x_g
    m->variant.device.accel_y_g = read16(0x2A) / 16384.0f;
    m->variant.device.accel_z_g = read16(0x2C) / 16384.0f;
    return m;
}

int16_t LIS3DHSensor::read16(uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, 2);
    return (int16_t)(Wire.read() | (Wire.read() << 8));
}

void LIS3DHSensor::write8(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(addr);
    Wire.write(reg); Wire.write(v);
    Wire.endTransmission();
}
