#include "LIS3DHSensor.h"
#include "../../mesh/generated/meshtastic/telemetry.pb.h"
#include "configuration.h"

LIS3DHSensor::LIS3DHSensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "LIS3DH") {}

int32_t LIS3DHSensor::runOnce() {
    if (!initSensor()) return -1;
    
    // ✅ DeviceMetrics STANDARD (sans accel)
    meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
    t.which_variant = meshtastic_Telemetry_device_metrics_tag;  // ✅ minuscule
    
    // Stockez X,Y,Z dans des champs libres (ex: voltage pour test)
    float x = read16(0x28) / 16384.0f;
    float y = read16(0x2A) / 16384.0f;
    float z = read16(0x2C) / 16384.0f;
    
    LOG_INFO("LIS3DH Seismic: X=%.3fg Y=%.3fg Z=%.3fg\n", x, y, z);
    
    // TODO: Ajouter custom protobuf pour motion ou utiliser debug text
    return 5000;
}

// initSensor() et read16/write8 IDENTIQUES (gardez-les)
