// SeismicTelemetry.h

#pragma once
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"

// Adresse I2C typique du LIS3DH (RAK1904)
// static constexpr uint8_t LIS3DH_ADDR = 0x18;
class SeismicTelemetryModule : public concurrency::OSThread, public ProtobufModule<meshtastic_Telemetry>
{

public:
    SeismicTelemetryModule();
    virtual ~SeismicTelemetryModule() = default;
    // meshService();

    void begin();
    void handle();

protected:
    // Gère le cycle de vie du thread (remplace handle)
    virtual int32_t runOnce() override;

    // Gère la réception de requêtes (CLI / App)
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *p) override;

private:
    bool m_hasLIS3DH = false;

    // Période de temps de la dernière lecture
    uint32_t m_lastSeismic;
    uint32_t m_lastHeartbeat = 0;

    // Seuil de détection pour le Jerk (en g/s)
    //   m_tolerance(0.05f),
    //   m_tolerance(0.025f),
    //   m_tolerance(0.005f),
    //   m_tolerance(0.001f),
    float m_tolerance = 0.001f;

    // Valeurs d'accélération (en g) lues au cycle précédent
    float m_xPrev = 0.0f, m_yPrev = 0.0f, m_zPrev = 0.0f;
    float curX = 0, curY = 0, curZ = 0;

    // Fonction d'envoi du paquet Telemetry Motion (conforme au proto Meshtastic)
    void sendTelemetryMotion(float dx, float dy, float dz,
                             float x, float y, float z, bool isAlert);
    void readSensor();
};
