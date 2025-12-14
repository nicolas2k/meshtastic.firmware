// SeismicModule.h
#pragma once

#include "MeshService.h"

class SeismicModule
{
public:
    SeismicModule(MeshService &service);

    void begin();
    void handle();

private:
    MeshService &m_service;
    uint32_t m_lastSeismic;
    float m_tolerance;
    float m_xPrev;
    float m_yPrev;
    float m_zPrev;

    void sendTelemetryMotion(float dx, float dy, float dz,
                             float x, float y, float z);
};
