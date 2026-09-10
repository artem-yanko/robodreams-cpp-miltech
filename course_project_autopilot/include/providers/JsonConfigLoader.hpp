#pragma once

#include "domain/types.hpp"
#include <string>
#include <vector>

class JsonConfigLoader {
public:
    explicit JsonConfigLoader(const std::string& configPath = "config.json", const std::string& ammoPath = "ammo.json");

    bool loadConfig(DroneConfig& config);
    bool loadAmmo(std::vector<AmmoParams>& ammoList);

private:
    std::string configPath;
    std::string ammoPath;
};
