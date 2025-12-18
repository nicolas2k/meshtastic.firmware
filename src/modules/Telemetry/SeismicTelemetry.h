// SeismicTelemetryModule.h

#pragma once
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"

// Utilisation de l'espace de noms pour la clarté
class SeismicTelemetryModule : public concurrency::OSThread, public ProtobufModule<meshtastic_Telemetry>
{
public:
    SeismicTelemetryModule();
    virtual ~SeismicTelemetryModule() = default;

    void begin();

protected:
    // Gère le cycle de vie du thread (remplace handle)
    virtual int32_t runOnce() override;
    
    // Gère la réception de requêtes (CLI / App)
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *p) override;

private:
    void sendTelemetryMotion(float dx, float dy, float dz, float x, float y, float z, bool isAlert);
    void readSensor();

    bool m_hasLIS3DH = false;
    uint32_t m_lastHeartbeat = 0;
    
    // Seuils et calculs
    float m_tolerance = 0.005f;
    float m_xPrev = 0.0f, m_yPrev = 0.0f, m_zPrev = 0.0f;
    float curX = 0, curY = 0, curZ = 0;
};
