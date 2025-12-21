// SeismicTelemetry.h

#pragma once
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include <vector>

/**
 * Structure légère pour le "Pull on Demand" des derniers événements
 */
struct SeismicEvent {
    uint32_t timestamp;
    float maxJerk;
};

class SeismicTelemetryModule : public concurrency::OSThread, public ProtobufModule<meshtastic_Telemetry>
{
public:
    SeismicTelemetryModule();
    virtual ~SeismicTelemetryModule() = default;

    void begin();

protected:
    // Boucle principale du thread
    virtual int32_t runOnce() override;

    // Réponse aux requêtes distantes (App / CLI)
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *p) override;

private:
    bool m_hasLIS3DH = false;

    // --- Paramètres configurables ---
    // SeismicBroadcastSecs : par défaut 3600s (1h), min 60s
    uint32_t m_broadcastIntervalSecs = 3600;

    // Votre réglage validé à 0.001f
    // float m_eventThreshold = 0.001f;
    float m_eventThreshold = 8.0f;

    // Seuil de mémorisation (un peu plus bas que l'alerte pour l'historique)
    float m_recordThreshold = 0.05f;

    uint32_t m_lastHeartbeat = 0;

    // --- Historique (Pull on Demand) ---
    std::vector<SeismicEvent> m_eventHistory;
    const size_t MAX_HISTORY = 10;

    // Valeurs de calcul
    float m_xPrev = 0.0f, m_yPrev = 0.0f, m_zPrev = 0.0f;
    float curX = 0, curY = 0, curZ = 0;

    void sendTelemetryMotion(float dx, float dy, float dz, bool isAlert);
    void readSensor();
    void recordEvent(float jerk);
};
