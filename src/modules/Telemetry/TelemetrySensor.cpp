// Dans la liste des sensors, après LIS3DHSensor
#ifdef USE_LIS3DH
sensors.emplace_back(new LIS3DHSensor());
#endif
