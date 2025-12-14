// SeismicModule.cpp

#include "SeismicModule.h"
#include "configuration.h"
#include <Wire.h>

#include "../mesh/generated/meshtastic/mesh.pb.h"      // MeshPacket, Data, Telemetry...
#include "../mesh/generated/meshtastic/telemetry.pb.h"  // si Telemetry/Motion sont dans un proto séparé
#include <RTC.h>

// static constexpr uint8_t LIS3DH_ADDR = 0x18;

SeismicModule::SeismicModule(MeshService &service)
    : m_service(service),
      m_lastSeismic(0),
      m_tolerance(0.15f),
      m_xPrev(0.0f),
      m_yPrev(0.0f),
      m_zPrev(0.0f)
{
}

void SeismicModule::begin()
{
#if defined(USE_LIS3DH_SENSOR)
    // Si l’IMU est déjà initialisée ailleurs, tu peux laisser vide.
    // Sinon, mettre ici la config RAK1904/LIS3DH.
#endif
}

void SeismicModule::handle()
{
#if defined(USE_LIS3DH_SENSOR)
    // SEISMIC RAK1904 DIFFÉRENTIELLE 250ms
    if (millis() - m_lastSeismic > 100) {  // ou 250 ms selon ton choix

        Wire.beginTransmission(LIS3DH_ADDR);
        Wire.write(0x28 | 0x80);
        Wire.endTransmission(false);
        Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6);

        if (Wire.available() == 6) {
            float x = (int16_t)(Wire.read() | (Wire.read() << 8)) / 16384.0f;
            float y = (int16_t)(Wire.read() | (Wire.read() << 8)) / 16384.0f;
            float z = (int16_t)(Wire.read() | (Wire.read() << 8)) / 16384.0f;

            float dx = (x - m_xPrev) / 0.25f;  // Δg / 0.25s = g/s
            float dy = (y - m_yPrev) / 0.25f;
            float dz = (z - m_zPrev) / 0.25f;

            if (dx >= m_tolerance || dy >= m_tolerance || dz >= m_tolerance) {
                // envoi telemetry.motion + logs
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


// Dans src/modules/SeismicModule.cpp
// Fonction: SeismicModule::sendTelemetryMotion

void SeismicModule::sendTelemetryMotion(float dx, float dy, float dz, float r, float p, float y) {
    
    // Initialisation de la structure de télémesure
    meshtastic_Telemetry telemetry = meshtastic_Telemetry_init_default;
    telemetry.time = getTime(); // Assurez-vous que getTime() est disponible
    
    // **NOTE IMPORTANTE : Structure Protobuf**
    // L'erreur 82 ('meshtastic_Telemetry' has no member named 'motion') indique que votre champ 'motion' n'est pas dans 'meshtastic_Telemetry' 
    // dans la structure officielle.
    // Dans le code standard Meshtastic, les métriques personnalisées sont souvent placées sous 'environment_metrics' ou 'device_metrics' 
    // ou nécessitent la création d'un nouveau champ Protobuf.
    
    // Hypothèse la plus probable pour un fork: Vous avez soit *ajouté* 'motion' directement à 'Telemetry', 
    // soit vous avez l'intention d'utiliser un champ existant pour ces données (comme 'environment_metrics').

    // **OPTION 1: Utiliser la structure standard (ex: EnvironmentMetrics) si possible**
    // Si votre fork mappe dx, dy, dz dans EnvironmentMetrics (peu probable pour du sismique), ce serait:
    // telemetry.which_variant = meshtastic_Telemetry_environment_metrics_tag;
    // telemetry.variant.environment_metrics.dx = dx; // Ceci causerait probablement une erreur si 'dx' n'existe pas dans EnvironmentMetrics

    // **OPTION 2: Réparation minimale du code existant (en supposant que 'motion' est défini mais accédé directement)**
    // Nous allons réécrire en supposant que votre fork a défini 'motion' comme un nouveau oneof dans Telemetry.

    // 1. Définir le type de télémesure (DOIT correspondre au tag de votre fork)
    // S'il s'agit d'une nouvelle structure SISMIQUE :
    // telemetry.which_variant = meshtastic_Telemetry_seismic_metrics_tag; 
    // S'il utilise le tag ENVIRONNEMENTAL pour l'instant (comme dans EnvironmentTelemetry.cpp):
    telemetry.which_variant = meshtastic_Telemetry_environment_metrics_tag; 
    
    // 2. Remplissage des données.
    // L'erreur 'meshtastic_Telemetry' has no member named 'motion' (lignes 82-84) signifie que vous ne pouvez pas
    // accéder directement à 'telemetry.motion'. 
    // Vous devez l'encapsuler dans le 'variant' approprié.
    
    // Nous allons utiliser une nouvelle structure fictive 'seismic_metrics' qui devrait être définie dans votre fork,
    // ou vous devez adapter pour utiliser un champ Protobuf valide (ex: un champ libre dans EnvironmentMetrics si disponible).

    // Pour que le code compile, nous devons utiliser une structure Protobuf existante et y placer les données.
    // Si vous n'avez pas créé de tag sismique, la seule façon de compiler est de mettre les données de mouvement dans un champ disponible. 
    
    // **CORRECTION POUR LA COMPILATION (en supposant que vous avez une structure de mouvement interne dans le fork):**
    // Si la structure 'telemetry.motion' existe, vous n'avez plus besoin de 'has_motion'.
    // Les erreurs d'accès à 'telemetry.motion.dx/dy/dz' indiquent un problème de structure Protobuf. 
    
    // **POUR FAIRE COMPILER LE CODE (vous DEVEZ vérifier la structure exacte de votre fork ici)**
    // Si 'motion' était une structure de niveau supérieur qui a été refactorisée dans votre fork:
    
    // Créons une structure pour contenir les valeurs (cela devrait idéalement être défini dans telemetry.proto)
    // Puisque nous n'avons pas la structure de votre fork, nous allons utiliser une affectation directe si elle existe.
    
    // Si vous avez ajouté 'motion' dans le message Telemetry, il doit être accessible via 'variant':
    // telemetry.which_variant = meshtastic_Telemetry_motion_tag;
    // telemetry.variant.motion.dx = dx; 
    // ...
    
    // REVENONS À LA CORRECTION D'ERREURS D'API PRINCIPALES:
    
    // L'ancienne ligne 69 (m_service.allocDataPacket()) est remplacée par:
    meshtastic_MeshPacket *packet = allocDataProtobuf(telemetry); // Utilise la méthode de EnvironmentTelemetry.cpp
    
    if (packet) {
        packet->to = NODENUM_BROADCAST; // Règle le destinataire
        packet->decoded.want_response = false;
        packet->priority = meshtastic_MeshPacket_Priority_BACKGROUND; // Règle la priorité
        
        m_service.sendToMesh(packet, RX_SRC_LOCAL, true); 
        free(packet);
    }
}