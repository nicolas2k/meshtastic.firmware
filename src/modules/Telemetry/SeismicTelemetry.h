// SeismicTelemetry.h
#pragma once

#include "ProtobufModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "../mesh/generated/meshtastic/mesh.pb.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"

class SeismicTelemetryModule : public ProtobufModule<meshtastic_Telemetry>
{
private:
    bool m_hasLIS3DH;

public:
    SeismicTelemetryModule();
    // meshService();

    void begin();
    void handle();

private:
    uint32_t m_lastSeismic;
    float m_tolerance;
    float m_xPrev;
    float m_yPrev;
    float m_zPrev;

    void sendTelemetryMotion(float dx, float dy, float dz,
                             float x, float y, float z);
};
