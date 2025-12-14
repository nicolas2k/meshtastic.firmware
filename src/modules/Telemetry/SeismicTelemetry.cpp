// SeismicTelemetryModule.cpp
#include "configuration.h"

#include "SeismicTelemetry.h"

#include "../mesh/generated/meshtastic/mesh.pb.h"      // MeshPacket, Data, Telemetry...
#include "../mesh/generated/meshtastic/telemetry.pb.h"  // si Telemetry/Motion sont dans un proto séparé

#include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "PowerTelemetry.h"
#include "Router.h"
#include "graphics/SharedUIDisplay.h"
#include "main.h"
#include "power.h"
#include "sleep.h"
#include "target_specific.h"

#include "EnvironmentTelemetry.h"
#include <Wire.h>
#include <RTC.h>
#include <cstdio>

// static constexpr uint8_t LIS3DH_ADDR = 0x18;

SeismicTelemetryModule::SeismicTelemetryModule()
    : ProtobufModule("SeismicTelemetryModule",
                     meshtastic_PortNum_TELEMETRY_APP,
                     &meshtastic_Telemetry_msg),
      m_lastSeismic(0),
      m_tolerance(0.15f),
      m_xPrev(0.0f),
      m_yPrev(0.0f),
      m_zPrev(0.0f),
      m_hasLIS3DH(false)
{
    // LOG_DEBUG("[SEISMIC] SeismicTelemetry: LIS3DH module");
}

void SeismicTelemetryModule::begin()
{
    LOG_INFO("[SEISMIC] SeismicTelemetry: begin()");

#if defined(USE_LIS3DH_SENSOR)
    // const uint8_t LIS3DH_ADDR = 0x18;

    // Ne pas refaire Wire.begin() ici, Meshtastic l’a déjà fait.
    // On vérifie juste que le capteur répond.
    Wire.beginTransmission(LIS3DH_ADDR);
    uint8_t err = Wire.endTransmission(true);   // true = stop condition

    if (err == 0) {
        m_hasLIS3DH = true;
        LOG_INFO("SeismicTelemetry: LIS3DH detected at 0x%02X", LIS3DH_ADDR);

        // Configuration minimale stable (à adapter si tu as déjà un code qui marchait)
        // Exemple : activer XYZ en 10 Hz
        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x20);                   // CTRL_REG1
        Wire.write(0b00100111);             // 10Hz, XYZ enable
        Wire.endTransmission(true);
    } else {
        m_hasLIS3DH = false;
        LOG_WARN("SeismicTelemetry: LIS3DH NOT found at 0x%02X (err=%d)", LIS3DH_ADDR, err);
    }
#endif
}

void SeismicTelemetryModule::handle()
{
    // LOG_DEBUG("[SEISMIC] SeismicTelemetry: handle()");

#if defined(USE_LIS3DH_SENSOR)
    if (!m_hasLIS3DH) return;

    // static const uint8_t LIS3DH_ADDR = 0x18;
    const uint32_t now = millis();

    // Limite la fréquence (à adapter si besoin)
    if (now - m_lastSeismic < 100) return;
    m_lastSeismic = now;

    // Lecture des 6 registres XYZ (0x28..0x2D)
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x28 | 0x80);       // auto-increment à partir de OUT_X_L
    if (Wire.endTransmission(false) != 0) {
        // Erreur I2C, on ne lit rien
        return;
    }

    if (Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6) != 6) {
        // Pas assez de données, on abandonne ce tour
        return;
    }

    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);

    float x = rawX / 16384.0f;
    float y = rawY / 16384.0f;
    float z = rawZ / 16384.0f;

    float dx = (x - m_xPrev) / 0.25f;
    float dy = (y - m_yPrev) / 0.25f;
    float dz = (z - m_zPrev) / 0.25f;

    m_xPrev = x;
    m_yPrev = y;
    m_zPrev = z;

    if (dx < m_tolerance && dy < m_tolerance && dz < m_tolerance) {
        return;
    }

    sendTelemetryMotion(dx, dy, dz, x, y, z);
#endif
}

// Dans src/modules/SeismicTelemetryModule.cpp
// Fonction: SeismicTelemetryModule::sendTelemetryMotion
void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz,
                                                 float x, float y, float z)
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.which_variant = meshtastic_Telemetry_motion_tag;
    m.time = getTime();

    m.variant.motion.dx = dx;
    m.variant.motion.dy = dy;
    m.variant.motion.dz = dz;

    meshtastic_MeshPacket *p = allocDataProtobuf(m);
    if (!p) return;

    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    service->sendToMesh(p, RX_SRC_LOCAL, true);

    char msg[64];
    snprintf(msg, sizeof(msg),
             "[SEISMIC] %.3f:%.3f:%.3f|%.1f:%.1f:%.1f",
             x, y, z, dx, dy, dz);
    LOG_INFO("%s", msg);
}
