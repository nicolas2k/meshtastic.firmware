#include "configuration.h"
#include "SeismicTelemetry.h"

#include "../mesh/generated/meshtastic/mesh.pb.h"      
#include "../mesh/generated/meshtastic/telemetry.pb.h"  

#include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "main.h"
#include "RTC.h"

#include <Wire.h>
#include <cmath>

extern NodeDB *nodeDB; 
extern MeshService *service; 
extern Router *router;

static constexpr float SENSITIVITY_2G = 1365.0f;
static constexpr float TIME_STEP = 0.100f; 

// CORRECTION 1 : Suppression du '&' car la macro meshtastic_Telemetry_fields l'inclut déjà
SeismicTelemetryModule::SeismicTelemetryModule() 
    : ProtobufModule<meshtastic_Telemetry>("seismic", meshtastic_PortNum_TELEMETRY_APP, meshtastic_Telemetry_fields), 
      m_hasLIS3DH(false), m_lastSeismic(0), m_tolerance(0.1f), 
      m_xPrev(0), m_yPrev(0), m_zPrev(0) 
{}

void SeismicTelemetryModule::begin() {
    Wire.begin();
    Wire.beginTransmission(0x18);
    Wire.write(0x0F); 
    if (Wire.endTransmission() == 0) {
        Wire.requestFrom(0x18, (uint8_t)1);
        if (Wire.read() == 0x33) {
            m_hasLIS3DH = true;
            Wire.beginTransmission(0x18);
            Wire.write(0x20); Wire.write(0x27); 
            Wire.endTransmission();
            Wire.beginTransmission(0x18);
            Wire.write(0x23); Wire.write(0x08); 
            Wire.endTransmission();
            LOG_INFO("LIS3DH détecté.");
        }
    }
}

void SeismicTelemetryModule::handle() {
    if (!m_hasLIS3DH || (millis() - m_lastSeismic < 100)) return;
    m_lastSeismic = millis();

    Wire.beginTransmission(0x18);
    Wire.write(0x28 | 0x80); 
    if (Wire.endTransmission() != 0) return;
    Wire.requestFrom(0x18, (uint8_t)6);
    if (Wire.available() < 6) return;

    int16_t xRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t yRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t zRaw = (int16_t)(Wire.read() | (Wire.read() << 8));

    float x = (float)(xRaw >> 4) / SENSITIVITY_2G;
    float y = (float)(yRaw >> 4) / SENSITIVITY_2G;
    float z = (float)(zRaw >> 4) / SENSITIVITY_2G;

    float dx = (x - m_xPrev) / TIME_STEP;
    float dy = (y - m_yPrev) / TIME_STEP;
    float dz = (z - m_zPrev) / TIME_STEP;

    LOG_INFO("[SEISMIC] Trigger: dx=%.3f dy=%.3f dz=%.3f", dx, dy, dz);

    sendTelemetryMotion(dx, dy, dz, x, y, z);

    meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
    t.which_variant = meshtastic_Telemetry_motion_tag;
    t.variant.motion.dx = dx;
    t.variant.motion.dy = dy;
    t.variant.motion.dz = dz;
    
    // CORRECTION 2 : Utilisation de RTC.getTimestamp() au lieu de getTime()
    t.time = RTC.getTimestamp(); 

    nodeDB->updateTelemetry(nodeDB->getNodeNum(), t); 

    m_xPrev = x; m_yPrev = y; m_zPrev = z;
}

// CORRECTION 3 : Utilisation de allocDataPacket() au lieu de allocPacket()
void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz, float x, float y, float z)
{
    meshtastic_Telemetry p = meshtastic_Telemetry_init_zero;
    p.which_variant = meshtastic_Telemetry_motion_tag;
    p.variant.motion.dx = dx;
    p.variant.motion.dy = dy;
    p.variant.motion.dz = dz;

    meshtastic_MeshPacket *pkg = allocDataPacket();
    pkg->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    pkg->to = 0xFFFFFFFF; 
    
    // CORRECTION 1 (bis) : Suppression du '&' devant la macro
    size_t size = pb_encode_to_bytes(pkg->decoded.payload.bytes, sizeof(pkg->decoded.payload.bytes), meshtastic_Telemetry_fields, &p);
    pkg->decoded.payload.size = size;

    LOG_INFO("Seismic TX...");
    
    // Utilisation de la valeur 0 pour SUCCESS (Embedded_ErrorCode_SUCCESS)
    ErrorCode res = router->send(pkg);

    if (res == 0) { 
        LOG_INFO("Packet sent successfully.");
    } else {
        LOG_WARN("Packet failed (Error: %d).", res);
    }
}