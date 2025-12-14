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
// #include "RTC.h"
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
    : m_service(service),
      m_lastSeismic(0),
      m_tolerance(0.15f),
      m_xPrev(0.0f),
      m_yPrev(0.0f),
      m_zPrev(0.0f)
{
}

void SeismicTelemetryModule::begin()
{
#if defined(USE_LIS3DH_SENSOR)
    // Si l’IMU est déjà initialisée ailleurs, tu peux laisser vide.
    // Sinon, mettre ici la config RAK1904/LIS3DH.
#endif
}

void SeismicTelemetryModule::handle()
{
#if defined(USE_LIS3DH_SENSOR)
    // Ton code original, mais en appelant sendTelemetryMotion()
    static const uint8_t LIS3DH_ADDR = 0x18;

    if (millis() - m_lastSeismic > 100) {  // ou 250 ms selon ton besoin
        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x28 | 0x80);
        Wire.endTransmission(false);
        Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6);

        if (Wire.available() == 6) {
            float x = (int16_t)(Wire.read() | (Wire.read() << 8)) / 16384.0f;
            float y = (int16_t)(Wire.read() | (Wire.read() << 8)) / 16384.0f;
            float z = (int16_t)(Wire.read() | (Wire.read() << 8)) / 16384.0f;

            float dx = (x - m_xPrev) / 0.25f;
            float dy = (y - m_yPrev) / 0.25f;
            float dz = (z - m_zPrev) / 0.25f;

            if (dx >= m_tolerance || dy >= m_tolerance || dz >= m_tolerance) {
                sendTelemetryMotion(dx, dy, dz, x, y, z);
            }

            m_xPrev = x;
            m_yPrev = y;
            m_zPrev = z;
        }

        m_lastSeismic = millis();
    }
#endif
}



// Dans src/modules/SeismicTelemetryModule.cpp
// Fonction: SeismicTelemetryModule::sendTelemetryMotion

void SeismicTelemetryModule::sendTelemetryMotion(float dx, float dy, float dz,
                                        float x, float y, float z)
{
    // 1) Préparer Telemetry comme pour l’environnement, mais avec la variante motion
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.which_variant = meshtastic_Telemetry_motion_tag;  // <== important
    m.time = getTime();                                 // même getTime() que les autres modules

    // Remplir la variante motion
    m.variant.motion.dx = dx;
    m.variant.motion.dy = dy;
    m.variant.motion.dz = dz;

    // 2) Créer le paquet comme le fait EnvironmentTelemetry (allocDataProtobuf)
    meshtastic_MeshPacket *p = allocDataProtobuf(m);    // même utilitaire
    if (!p) return;

    // 3) Paramètres d’envoi similaires
    p->to = NODENUM_BROADCAST;         // ou un dest précis si tu veux
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    // 4) Envoyer sur le mesh (pas au téléphone ici)
    service->sendToMesh(p, RX_SRC_LOCAL, true);

    // 5) Logs texte pour debug (comme ton code initial)
    char msg[64];
    snprintf(msg, sizeof(msg),
             "[SEISMIC] %.3f:%.3f:%.3f|%.1f:%.1f:%.1f",
             x, y, z, dx, dy, dz);
    printf("%s\n", msg);
    LOG_INFO("%s", msg);
}
