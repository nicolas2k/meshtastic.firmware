#include "TelemetrySensor.h"
#include "configuration.h"
#include "../../mesh/MeshService.h"

TelemetrySensor *TelemetrySensor::instance = nullptr;
Meshtastic::Telemetry::EnvironmentMetrics *envMetrics = nullptr;

TelemetrySensor::TelemetrySensor() : Module("TelemetrySensor", Module::defaultTaskQueue) {
    instance = this;
}

void TelemetrySensor::setup() {
    if (!lis.begin(LIS3DH_ADDR)) {  // ✅ 0x19 directement pour RAK1904 plutôt 0x18
        LOG_WARN("LIS3DH RAK1904 non trouvé\n");
        return;
    }
    lis.setRange(LIS3DH_RANGE_4_G);
    lis.setDataRate(LIS3DH_DATARATE_50_HZ);
    lis3dh_ready = true;
    LOG_INFO("LIS3DH RAK1904 initialisé\n");
}

int32_t TelemetrySensor::runOnce() {
    if (!lis3dh_ready || !telemetry) return 5000;
    
    envMetrics = telemetry->envMetrics;
    
    if (millis() - last_accel_update > ACCEL_UPDATE_INTERVAL) {
        sensors_event_t accel;
        if (lis.getEvent(&accel)) {
            envMetrics->acceleration.x = accel.acceleration.x;
            envMetrics->acceleration.y = accel.acceleration.y;
            envMetrics->acceleration.z = accel.acceleration.z;
            LOG_DEBUG("Accel: X=%.2f Y=%.2f Z=%.2f\n", 
                     envMetrics->acceleration.x, 
                     envMetrics->acceleration.y, 
                     envMetrics->acceleration.z);
        }
        last_accel_update = millis();
    }
    
    return ACCEL_UPDATE_INTERVAL;
}
