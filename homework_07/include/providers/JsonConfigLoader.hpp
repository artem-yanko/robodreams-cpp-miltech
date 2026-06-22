#pragma once

#include "interfaces/IConfigLoader.hpp"
#include <string>

class JsonConfigLoader : public IConfigLoader {
public:
    explicit JsonConfigLoader(const std::string& configPath = "config.json", const std::string& ammoPath = "ammo.json");

    bool loadConfig(DroneConfig& config) override;
    bool loadAmmo(AmmoParams*& ammoList, int& ammoCount) override;

private:
    std::string configPath_;
    std::string ammoPath_;
};
