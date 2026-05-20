#pragma once

#include "domain/types.hpp"

class IConfigLoader {
public:
    virtual bool loadConfig(DroneConfig& config) = 0;
    virtual bool loadAmmo(AmmoParams*& ammoList, int& ammoCount) = 0;
    virtual ~IConfigLoader() {}
};