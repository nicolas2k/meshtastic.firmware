// SeismicTelemetry.cpp

#include "configuration.h"  // Contient LIS3DH_ADDR
#include "SeismicTelemetry.h"
#include "MeshService.h"
// #include "MeshPackets.h" // Indispensable pour allocPacket()
#include "NodeDB.h"
#include "Router.h"
#include <Wire.h>
#include <RTC.h>
#include <pb_encode.h>


static constexpr float SENSITIVITY_2G = 1365.0f;
static constexpr float TIME_STEP = 0.100f; // 100ms (10Hz)

SeismicTelemetryModule::SeismicTelemetryModule()
    : concurrency::OSThread("Seismic"),
      ProtobufModule("Seismic", meshtastic_PortNum_TELEMETRY_APP, &meshtastic_Telemetry_msg)
{
    // On règle l'intervalle d'exécution du thread (100ms pour le polling)
    setIntervalFromNow(100);
}

void SeismicTelemetryModule::begin()
{
    LOG_INFO("[Seismic] Démarrage avec LIS3DH_ADDR: 0x%02X", LIS3DH_ADDR);
    Wire.begin();

    // Test de présence sur le bus I2C
    Wire.beginTransmission(LIS3DH_ADDR);
    if (Wire.endTransmission() == 0) {
        m_hasLIS3DH = true;

        // --- Configuration registre LIS3DH ---
        // CTRL_REG4: FS ±2g, High Resolution
        Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x23); Wire.write(0x08); Wire.endTransmission();
        // CTRL_REG1: 10Hz, Normal Power, All axes ON
        Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x20); Wire.write(0x27); Wire.endTransmission();

        LOG_INFO("[Seismic] Capteur détecté et configuré (Seuil: %.4f)", m_eventThreshold);
    } else {
        LOG_ERROR("[Seismic] ERREUR: LIS3DH non trouvé !");
    }
}

int32_t SeismicTelemetryModule::runOnce()
{
    if (!m_hasLIS3DH) return 5000; // Réessayer plus tard si échec

    readSensor();

    // Calcul du Jerk (variation d'accélération)
    float dx = (curX - m_xPrev) / TIME_STEP;
    float dy = (curY - m_yPrev) / TIME_STEP;
    float dz = (curZ - m_zPrev) / TIME_STEP;
    float maxJerk = max(fabs(dx), max(fabs(dy), fabs(dz)));

    uint32_t now = millis();

    // 1. GESTION ALERTE (onEventDetected)
    if (maxJerk > m_eventThreshold) {
        LOG_WARN("[Seismic] Jerk: %.4f, dx:%.4f, dy:%.4f, dz:%.4f", maxJerk, dx, dy, dz);
        sendTelemetryMotion(dx, dy, dz, true);
        m_lastHeartbeat = now; // Réinitialise le délai heartbeat
    }
    // 2. GESTION HEARTBEAT (SeismicBroadcastSecs)
    else if (now - m_lastHeartbeat >= (max((uint32_t)60, m_broadcastIntervalSecs) * 1000)) {
        LOG_INFO("[Seismic] Heartbeat cyclique");
        sendTelemetryMotion(0, 0, 0, false);
        m_lastHeartbeat = now;
    }

    m_xPrev = curX; m_yPrev = curY; m_zPrev = curZ;
    return 100; // Prochain scan dans 100ms
}

void SeismicTelemetryModule::recordEvent(float jerk) {
    SeismicEvent ev = {getTime(), jerk};
    m_eventHistory.push_back(ev);
    if (m_eventHistory.size() > MAX_HISTORY) {
        m_eventHistory.erase(m_eventHistory.begin());
    }
}

void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz, bool isAlert)
{
    // --- 1. ENVOI DE LA TELEMETRIE (PORT 6) ---
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.which_variant = meshtastic_Telemetry_motion_tag;
    m.time = getTime();
    m.variant.motion.dx = dx;
    m.variant.motion.dy = dy;
    m.variant.motion.dz = dz;

    // Cette méthode est héritée de ProtobufModule, elle fonctionne dans ton projet
    meshtastic_MeshPacket *p = allocDataProtobuf(m);
    if (p) {
        p->to = NODENUM_BROADCAST;
        // On force le port 6 pour que l'app Android reconnaisse la télémétrie
        p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
        // Le channel index commence à 0. Pour le canal "7" (si c'est bien son index) :
        p->channel = 7;
        p->priority = isAlert ? meshtastic_MeshPacket_Priority_HIGH : meshtastic_MeshPacket_Priority_BACKGROUND;
        LOG_DEBUG("[Seismic] Sending on channel index: %d", p->channel);

        service->sendToMesh(p, RX_SRC_LOCAL, true);
    }

    // --- 2. ENVOI DU TEXTE (PORT 1) SI ALERTE ---
    if (isAlert) {
char alertText[64];
        snprintf(alertText, sizeof(alertText), "SEISME detecte ! Jerk: %.2f", dz);

        // On crée un paquet "bidon" pour obtenir la mémoire
        meshtastic_Telemetry dummy = meshtastic_Telemetry_init_zero;
        meshtastic_MeshPacket *tp = allocDataProtobuf(dummy);

        if (tp) {
            tp->to = NODENUM_BROADCAST;
            // On change le port pour dire que c'est du TEXTE
            tp->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
            // Le channel index commence à 0. Pour le canal "7" (si c'est bien son index) :
            tp->channel = 7;
            tp->priority = meshtastic_MeshPacket_Priority_HIGH;
            LOG_DEBUG("[Seismic] Sending on channel index: %d", tp->channel);

            // On remplace le contenu par ton texte
            size_t len = strlen(alertText);
            tp->decoded.payload.size = len;
            memcpy(tp->decoded.payload.bytes, alertText, len);

            service->sendToMesh(tp, RX_SRC_LOCAL, true);
        }
    }
}

bool SeismicTelemetryModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *p) {
    LOG_INFO("[Seismic] Pull on Demand reçu de 0x%08x", mp.from);
    readSensor();
    sendTelemetryMotion(0, 0, 0, false);
    return true;
}

void SeismicTelemetryModule::readSensor() {
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x28 | 0x80);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6) == 6) {
        int16_t x = Wire.read() | (Wire.read() << 8);
        int16_t y = Wire.read() | (Wire.read() << 8);
        int16_t z = Wire.read() | (Wire.read() << 8);
        curX = x / SENSITIVITY_2G;
        curY = y / SENSITIVITY_2G;
        curZ = z / SENSITIVITY_2G;
    }
}
