// SeismicTelemetryModule.cpp
#include "configuration.h"

#include "SeismicTelemetry.h"

// Fichiers Protobuf requis
#include "../mesh/generated/meshtastic/mesh.pb.h"      // MeshPacket, Data, Telemetry...
#include "../mesh/generated/meshtastic/telemetry.pb.h"  // si Telemetry/Motion sont dans un proto séparé

// Inclusions standard Meshtastic
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
#include "Configuration.h"

#include <Wire.h> // Bus I2C
#include <RTC.h>
#include <cstdio>
#include <cmath> // Pour fabs()

// *** DÉCLARATIONS GLOBALES ***

extern meshtastic_MyNodeInfo &myNodeInfo; 
extern meshtastic_DeviceState devicestate; 
extern NodeDB *nodeDB; 
extern MeshService *service; // Requis pour refreshLocalMeshNode() et sendToMesh()
// Déclaration du helper Protobuf qui est une fonction globale ou de MeshService (assumons global ici)
extern meshtastic_MeshPacket *allocDataProtobuf(meshtastic_Telemetry &t);

// ***************************************

// Facteur de conversion pour la plage ±2g en mode Haute Résolution (12 bits)
// 1365.0f LSB/g est la valeur standard.
static constexpr float SENSITIVITY_2G = 1365.0f;
// Intervalle de temps pour l'ODR 10 Hz
static constexpr float TIME_STEP = 0.100f; // 100 ms


SeismicTelemetryModule::SeismicTelemetryModule()
    : ProtobufModule("SeismicTelemetryModule",
                     meshtastic_PortNum_TELEMETRY_APP, // Utilisation du port Telemetry
                     &meshtastic_Telemetry_msg),
      m_hasLIS3DH(false),
      m_lastSeismic(0),
      // Seuil Jerk (g/s). 0.025 g/s est une priorité BACKGROUND pour la sismicité.
      m_tolerance(0.025f),
      m_xPrev(0.0f),
      m_yPrev(0.0f),
      m_zPrev(0.0f)
{
    // LOG_DEBUG("[SEISMIC] SeismicTelemetry: LIS3DH module");
}

void SeismicTelemetryModule::begin()
{
    LOG_INFO("[SEISMIC] SeismicTelemetry: begin()");

#if defined(USE_LIS3DH_SENSOR)
    // Vérification I2C
    Wire.beginTransmission(LIS3DH_ADDR);
    uint8_t err = Wire.endTransmission(true);

    if (err == 0) {
        m_hasLIS3DH = true;
        LOG_INFO("SeismicTelemetry: LIS3DH detected at 0x%02X", LIS3DH_ADDR);

        // --- Configuration I2C pour la Haute Sensibilité et l'Interruption ---

        // 1. Définir la Plage de mesure à +/- 2g (CTRL_REG4: 0x23)
        // 0x18 = ±2g (FS=00), Haute Résolution (HR=1)
        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x23); Wire.write(0x18);
        Wire.endTransmission(true);

        // 2. Définir l'ODR à 10 Hz (CTRL_REG1: 0x20)
        // 0x27 = ODR 10Hz (0010), XYZ enable (111)
        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x20); Wire.write(0x27);
        Wire.endTransmission(true);

        // 3. Définir le Seuil (Trigger) à 15 LSB (~0.015g), nécessaire pour les interruptions
        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x32); Wire.write(0x0F);                   // 0x0F = 15 LSB
        Wire.endTransmission(true);

        // 4. Configuration des axes pour l'Interruption 1 (INT1_CFG: 0x30)
        // 0x2A = Détection sur X, Y et Z ("OR" logique)
        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x30); Wire.write(0x2A);
        Wire.endTransmission(true);

        // 5. Acheminer l'interruption vers la broche INT1 (CTRL_REG3: 0x22)
        // 0x40 = Active la détection d'événement sur INT1
        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x22); Wire.write(0x40);
        Wire.endTransmission(true);

        LOG_INFO("[SEISMIC] LIS3DH Configuré: ±2g, 10Hz, Seuil %f g/s.", m_tolerance);

    } else {
        m_hasLIS3DH = false;
        LOG_WARN("SeismicTelemetry: LIS3DH NOT found at 0x%02X (err=%d)", LIS3DH_ADDR, err);
    }
#endif
}


// Fonction d'envoi du paquet Telemetry Motion (conforme au proto Meshtastic)
// Nous n'utilisons que dx, dy, dz pour le paquet Protobuf motion, x, y, z sont des informations de contexte.
void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz,
                                                 float x, float y, float z)
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
    }

    // Configuration des champs du MeshPacket
    p->to = NODENUM_BROADCAST;
    // La priorité est définie par l'énumérateur standard
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    // Le champ decoded.want_response est géré par allocDataProtobuf
    
    service->sendToMesh(p, RX_SRC_LOCAL, true);
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

    // --- Lecture des 6 registres XYZ ---
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x28 | 0x80); // OUT_X_L (0x28) + auto-increment (0x80)
    if (Wire.endTransmission(false) != 0) { return; }
    if (Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6) != 6) { return; }

    // Lecture et reconstruction int16_t
    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);

    // Conversion en 'g' avec la sensibilité correcte (±2g: 1365.0f LSB/g)
    float x = rawX / SENSITIVITY_2G;
    float y = rawY / SENSITIVITY_2G;
    float z = rawZ / SENSITIVITY_2G;

    // Calcul du JERK (Taux de Changement d'Accélération) en g/s
    float dx = (x - m_xPrev) / TIME_STEP;
    float dy = (y - m_yPrev) / TIME_STEP;
    float dz = (z - m_zPrev) / TIME_STEP;

    // Test par rapport à la tolérance du Jerk (m_tolerance en g/s)
    if (fabs(dx) < m_tolerance && fabs(dy) < m_tolerance && fabs(dz) < m_tolerance) {
        m_xPrev = x;
        m_yPrev = y;
        m_zPrev = z;
        return;
    }

    char msg[64];
    snprintf(msg, sizeof(msg),
             "[SEISMIC TRIGGER] ABSOLU: %.3f:%.3f:%.3f | JERK (g/s): %.3f:%.3f:%.3f",
             x, y, z, dx, dy, dz);
    LOG_INFO("%s", msg);

    // *** PROCESSUS D'ENREGISTREMENT ET DE DIFFUSION ***
    // *** PROCESSUS D'ENREGISTREMENT ET DE DIFFUSION ***

// 1. Envoi immédiat du paquet de télémétrie (Portnum 6)
    sendTelemetryMotion(dx, dy, dz, x, y, z);

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

    // --- Réarmement de l'Interruption (ESSENTIEL) ---
    // Lecture du registre INT1_SRC (0x31) pour effacer l'état d'interruption dans le LIS3DH.
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x31);                   // INT1_SRC (registre source de l'interruption)
    Wire.endTransmission(false);

    Wire.requestFrom(LIS3DH_ADDR, 1);
    // Le fait de lire l'octet réarme la broche INT1 pour la prochaine détection.
    if (Wire.available()) {
        Wire.read();
    }

#endif
}


// Fonction d'envoi du paquet Meshtastic (conforme au proto)
void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz,
                                                 float x, float y, float z)
{
    // *** DEBUT DE LA SECTION PROTOBUF ***
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    // Indique que le type de télémétrie est "motion"
    m.which_variant = meshtastic_Telemetry_motion_tag;
    m.time = getTime();

    // On utilise dx, dy, dz pour envoyer le Jerk (g/s)
    m.variant.motion.dx = dx;
    m.variant.motion.dy = dy;
    m.variant.motion.dz = dz;

    meshtastic_MeshPacket *p = allocDataProtobuf(m);
    if (!p) return;

    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;

    // CORRECTION: Remplacement de l'identifiant non défini par sa valeur entière (2)
    // 2 correspond à meshtastic_MeshPacket_Priority_LOW
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    service->sendToMesh(p, RX_SRC_LOCAL, true);
    // *** FIN DE LA SECTION PROTOBUF ***
}
