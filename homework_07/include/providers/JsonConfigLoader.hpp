#pragma once

#include "interfaces/IConfigLoader.hpp"

class JsonConfigLoader : public IConfigLoader {
public:
    bool loadConfig(DroneConfig& config) override;
    bool loadAmmo(AmmoParams*& ammoList, int& ammoCount) override;
};