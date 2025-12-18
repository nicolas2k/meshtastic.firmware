// SeismicTelemetryModule.cpp

#include "configuration.h"
#include "SeismicTelemetry.h"

// Fichiers Protobuf requis
// #include "../mesh/generated/meshtastic/mesh.pb.h"
// #include "../mesh/generated/meshtastic/telemetry.pb.h"

// Inclusions standard Meshtastic
// #include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
// #include "PowerFSM.h"
// #include "PowerTelemetry.h"
#include "Router.h"
// #include "graphics/SharedUIDisplay.h"
// #include "main.h"
// #include "power.h"
// #include "sleep.h"
// #include "target_specific.h"
// #include "EnvironmentTelemetry.h"
// #include "error.h"

#include <Wire.h> // Bus I2C
#include <RTC.h>
// #include <cstdio>
// #include <cmath> // Pour fabs()

// *** DÉCLARATIONS GLOBALES ***
extern meshtastic_MyNodeInfo &myNodeInfo;
extern meshtastic_DeviceState devicestate;
extern NodeDB *nodeDB;
extern MeshService *service;
// ***************************************

// static constexpr uint8_t LIS3DH_ADDR = 0x18;
// Facteur de conversion pour la plage ±2g en mode Haute Résolution (12 bits)
static constexpr float SENSITIVITY_2G = 1365.0f;
// Intervalle de temps pour l'ODR 10 Hz
static constexpr float TIME_STEP = 0.100f; // 100 ms

// Déclaration de la fonction Protobuf Helper (doit être définie dans un autre fichier)
// Nous laissons cette déclaration ici en dernier recours pour aider l'éditeur de liens.
extern meshtastic_MeshPacket *allocDataProtobuf(meshtastic_Telemetry &t);


SeismicTelemetryModule::SeismicTelemetryModule()
    : concurrency::OSThread("Seismic"),
      ProtobufModule("Seismic", meshtastic_PortNum_TELEMETRY_APP, &meshtastic_Telemetry_msg)
{
    // On règle l'intervalle par défaut du thread à 100ms pour la détection sismique
    setIntervalFromNow(1/TIME_STEP);
}

void SeismicTelemetryModule::begin()
{
    LOG_INFO("[Seismic] SeismicTelemetry: begin()");

#if defined(USE_LIS3DH_SENSOR)
    Wire.begin();
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x0F); // WHO_AM_I
    uint8_t err = Wire.endTransmission(true);

    if (err == 0) {
        LOG_INFO("SeismicTelemetry: LIS3DH detected at 0x%02X", LIS3DH_ADDR);

        // Configuration I2C LIS3DH
        Wire.requestFrom(LIS3DH_ADDR, (uint8_t)1);
        if (Wire.read() == 0x33) {

            // 1. CTRL_REG4 (0x23) : Full Scale ±2g (plus sensible), High Resolution ON
            // Bits: BDU(0) | BLE(0) | FS(00) | HR(1) | ST(00) | SIM(0) => 0x08
            // Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x23); Wire.write(0x18); Wire.endTransmission(true);
            Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x23); Wire.write(0x08); Wire.endTransmission(true);

            // 2. CTRL_REG1 (0x20) : 10Hz, Power Normal, All Axes ON
            // Bits: ODR(0010) | LPen(0) | Zen(1) | Yen(1) | Xen(1) => 0x27
            // Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x20); Wire.write(0x27); Wire.endTransmission(true);
            Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x20); Wire.write(0x27); Wire.endTransmission(true);

            // 3. INT1_THS (0x32) : Seuil de détection
            // Valeur entre 0x01 (max sensibilité) et 0x7F. 0x02 = ~32mg
            // Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x32); Wire.write(0x0F); Wire.endTransmission(true);
            Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x32); Wire.write(0x02); Wire.endTransmission(true);

            // 4. INT1_CFG (0x30) : Configurer l'événement (OR de X High, Y High, Z High)
            // Bits: AOI(0) | 6D(0) | ZHIE(1) | ZLIE(0) | YHIE(1) | YLIE(0) | XHIE(1) | XLIE(0) => 0x2A
            // Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x30); Wire.write(0x2A); Wire.endTransmission(true);
            Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x30); Wire.write(0x2A); Wire.endTransmission(true);

            // 5. CTRL_REG3 (0x22) : Mapper l'interruption IA1 sur la pin INT1
            // Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x22); Wire.write(0x40); Wire.endTransmission(true);
            Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x22); Wire.write(0x40); Wire.endTransmission(true);
        }

        LOG_INFO("[Seismic] LIS3DH Configuré: ±2g, %fHz, Seuil %f g/s.", 1/TIME_STEP, m_tolerance);
        m_hasLIS3DH = true;

    } else {
        LOG_WARN("SeismicTelemetry: LIS3DH NOT found at 0x%02X (err=%d)", LIS3DH_ADDR, err);
        m_hasLIS3DH = false;
    }
#endif
}


// *** DÉFINITION UNIQUE DE LA FONCTION D'ENVOI ***
void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz,
                                                 float x, float y, float z, bool isAlert)
{
    // *** DEBUT DE LA SECTION PROTOBUF ***
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;

    m.which_variant = meshtastic_Telemetry_motion_tag;
    m.time = getTime();

    // On utilise dx, dy, dz pour envoyer le Jerk (g/s)
    m.variant.motion.dx = dx;
    m.variant.motion.dy = dy;
    m.variant.motion.dz = dz;

    // Utilisation de la fonction helper pour créer le MeshPacket
    meshtastic_MeshPacket *p = allocDataProtobuf(m);
    if (!p) {
        // Correction de LOG_ERR en LOG_ERROR
        LOG_ERROR("Erreur d'allocation de MeshPacket pour la télémétrie.");
        return;
    } else {
        LOG_WARN("[Seismic] shaked !");
    }

    // Configuration des champs du MeshPacket
    p->to = NODENUM_BROADCAST;
    // La priorité est définie par l'énumérateur standard
    // p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
    p->priority = isAlert ? meshtastic_MeshPacket_Priority_HIGH : p->priority = isAlert ? meshtastic_MeshPacket_Priority_HIGH : meshtastic_MeshPacket_Priority_BACKGROUND; //
    // Le champ decoded.want_response est géré par allocDataProtobuf
    service->sendToMesh(p, RX_SRC_LOCAL, true);

    // Log de l'envoi
    ErrorCode res = router->send(p);
    if (res == 0) {
        LOG_INFO("Seismic Packet submitted to Router successfully (Error: %d).", res);
    } else {
        LOG_WARN("Seismic Packet FAILED to submit to Router (Error: %d).", res);
        // Si l'erreur est liée à l'AirUtil (qui ne retourne pas d'erreur, mais un WARN),
        // le log WARN sera affiché APRES cette ligne.
    }


    // *** FIN DE LA SECTION PROTOBUF ***
}


void SeismicTelemetryModule::handle()
{
#if defined(USE_LIS3DH_SENSOR)
    if (!m_hasLIS3DH) return;

    const uint32_t now = millis();

    // Fréquence de lecture limitée à 100 ms (10 Hz)
    if (now - m_lastSeismic < 100) return;
    m_lastSeismic = now;

    // --- Lecture et calculs Jerk ---
    Wire.beginTransmission(LIS3DH_ADDR); Wire.write(0x28 | 0x80);
    if (Wire.endTransmission(false) != 0) { return; }
    if (Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6) != 6) { return; }

    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);

    float x = rawX / SENSITIVITY_2G;
    float y = rawY / SENSITIVITY_2G;
    float z = rawZ / SENSITIVITY_2G;

    float dx = (x - m_xPrev) / TIME_STEP;
    float dy = (y - m_yPrev) / TIME_STEP;
    float dz = (z - m_zPrev) / TIME_STEP;

    // Test par rapport à la tolérance du Jerk
    if (fabs(dx) < m_tolerance && fabs(dy) < m_tolerance && fabs(dz) < m_tolerance) {
        m_xPrev = x;
        m_yPrev = y;
        m_zPrev = z;
        return;
    }

    // Le seuil est dépassé : Log pour l'utilisateur
    char msg[64];
    snprintf(msg, sizeof(msg),
             "[SEISMIC TRIGGER] ABSOLU: %.3f:%.3f:%.3f | JERK (g/s): %.3f:%.3f:%.3f",
             x, y, z, dx, dy, dz);
    LOG_INFO("%s", msg);


    // *** PROCESSUS D'ENREGISTREMENT ET DE DIFFUSION ***

    // 1. Envoi immédiat du paquet de télémétrie (Portnum 6)
    sendTelemetryMotion(dx, dy, dz, x, y, z, true);

    // 2. Mise à jour de la base de données locale (pour la persistance et le NodeInfo)
    meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
    t.which_variant = meshtastic_Telemetry_motion_tag;
    t.variant.motion.dx = dx;
    t.variant.motion.dy = dy;
    t.variant.motion.dz = dz;
    t.time = getTime();

    nodeDB->updateTelemetry(nodeDB->getNodeNum(), t);
    nodeDB->saveToDisk();

    // 3. Mise à jour de la structure locale pour le timer (NodeInfo)
    service->refreshLocalMeshNode();

    // *************************************************************************
    // Stockage pour la prochaine itération
    m_xPrev = x;
    m_yPrev = y;
    m_zPrev = z;

    // --- Réarmement de l'Interruption (inchangé) ---
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x31);
    Wire.endTransmission(false);

    Wire.requestFrom(LIS3DH_ADDR, 1);
    if (Wire.available()) {
        Wire.read();
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


// #if defined(USE_LIS3DH_SENSOR)
// // L'instance globale suffit, ProtobufModule gère l'enregistrement
// SeismicTelemetryModule *seismicTelemetryModule = new SeismicTelemetryModule(); 
// #endif

