#pragma once

#include "domain/types.hpp"
#include <vector>

class IConfigLoader {
public:
    virtual bool loadConfig(DroneConfig& config) = 0;
    virtual bool loadAmmo(std::vector<AmmoParams>& ammoList) = 0;
    virtual ~IConfigLoader() {}
};
