// SeismicTelemetryModule.cpp

#include "configuration.h"
#include "SeismicTelemetry.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include <Wire.h> // Bus I2C
#include <RTC.h>

// static constexpr uint8_t LIS3DH_ADDR = 0x18;
static constexpr float SENSITIVITY_2G = 1365.0f;
static constexpr float TIME_STEP = 0.100f; // 10Hz

SeismicTelemetryModule::SeismicTelemetryModule()
    : concurrency::OSThread("Seismic"),
      ProtobufModule("Seismic", meshtastic_PortNum_TELEMETRY_APP, &meshtastic_Telemetry_msg)
{
    // On règle l'intervalle par défaut du thread à 100ms pour la détection sismique
    setIntervalFromNow(100);
}

void SeismicTelemetryModule::begin() {
#if defined(USE_LIS3DH_SENSOR)
    Wire.beginTransmission(LIS3DH_ADDR);
    if (Wire.endTransmission() == 0) {
        m_hasLIS3DH = true;
        // Configuration optimisée (Mode HR, 10Hz, Axes XYZ)
        Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x23); Wire.write(0x18); Wire.endTransmission();
        Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x20); Wire.write(0x27); Wire.endTransmission();
        LOG_INFO("[Seismic] LIS3DH Ready");
    }
#endif
}

// C'est ici que la magie opère (Thread Meshtastic)
int32_t SeismicTelemetryModule::runOnce() {
    if (!m_hasLIS3DH) return 1000; // Si pas de capteur, revérifier plus tard

    readSensor();

    float dx = (curX - m_xPrev) / TIME_STEP;
    float dy = (curY - m_yPrev) / TIME_STEP;
    float dz = (curZ - m_zPrev) / TIME_STEP;

    uint32_t now = millis();

    // 1. Logique d'ALERTE (Sismique)
    if (fabs(dx) > m_tolerance || fabs(dy) > m_tolerance || fabs(dz) > m_tolerance) {
        LOG_WARN("[Seismic] Movement detected!");
        sendTelemetryMotion(dx, dy, dz, curX, curY, curZ, true);
        m_lastHeartbeat = now; // On décale le prochain heartbeat
    } 
    // 2. Logique de HEARTBEAT (Toutes les 60 secondes)
    else if (now - m_lastHeartbeat >= 60000) {
        LOG_INFO("[Seismic] Sending Heartbeat");
        sendTelemetryMotion(0, 0, 0, curX, curY, curZ, false);
        m_lastHeartbeat = now;
    }

    m_xPrev = curX; m_yPrev = curY; m_zPrev = curZ;

    return 100; // Relance le thread dans 100ms
}

// Répond aux demandes CLI (ex: meshtastic --get telemetry)
bool SeismicTelemetryModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *p) {
    LOG_INFO("[Seismic] Request received from 0x%08x", mp.from);
    readSensor();
    sendTelemetryMotion(0, 0, 0, curX, curY, curZ, false);
    return true; 
}

void SeismicTelemetryModule::readSensor() {
    Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x28 | 0x80); Wire.endTransmission(false);
    Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6);
    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);
    curX = rawX / SENSITIVITY_2G;
    curY = rawY / SENSITIVITY_2G;
    curZ = rawZ / SENSITIVITY_2G;
}

void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz, float x, float y, float z, bool isAlert)
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.which_variant = meshtastic_Telemetry_motion_tag;
    m.time = getTime();
    m.variant.motion.dx = dx;
    m.variant.motion.dy = dy;
    m.variant.motion.dz = dz;

    meshtastic_MeshPacket *p = allocDataProtobuf(m);
    if (!p) {
        LOG_ERROR("Allocation failure");
        return;
    }

    p->to = NODENUM_BROADCAST;
    
    // Utilisation de l'argument isAlert pour définir la priorité
    p->priority = isAlert ? meshtastic_MeshPacket_Priority_HIGH : meshtastic_MeshPacket_Priority_BACKGROUND;

    service->sendToMesh(p, RX_SRC_LOCAL, true); 
}

#if defined(USE_LIS3DH_SENSOR)
SeismicTelemetryModule *seismicTelemetryModule = new SeismicTelemetryModule();
#endif

