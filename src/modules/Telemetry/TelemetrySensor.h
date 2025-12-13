#pragma once

#include "Module.h"
#include "../../mesh/generated/meshtastic/telemetry.pb.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

class TelemetrySensor : public Module {
private:
    Adafruit_LIS3DH lis = Adafruit_LIS3DH();
    bool lis3dh_ready = false;
    uint32_t last_accel_update = 0;
    static const uint32_t ACCEL_UPDATE_INTERVAL = 5000; // 5s

public:
    TelemetrySensor();
    virtual int32_t runOnce() override;
    virtual bool wantPeriodicRender() override { return false; }
    
    void setup() override;
    
    static TelemetrySensor *instance;
};

// Variables globales pour Meshtastic
extern Meshtastic::Telemetry::EnvironmentMetrics *envMetrics;
