#include "providers/JsonConfigLoader.hpp"
#include "utils/json.hpp"
#include "utils/logger.hpp"

#include <fstream>
#include <string>

using json = nlohmann::json;

JsonConfigLoader::JsonConfigLoader(const std::string& configPath, const std::string& ammoPath)
    : configPath(configPath), ammoPath(ammoPath) {
}

bool JsonConfigLoader::loadConfig(DroneConfig& config) {
    std::ifstream configFile(configPath);
    if (!configFile) {
        ERROR_LOG("Failed to open config file \"" << configPath << "\"");
        return false;
    }

    json configJson;
    configFile >> configJson;

    config.startPos.x = configJson["drone"]["position"]["x"];
    config.startPos.y = configJson["drone"]["position"]["y"];
    config.altitude = configJson["drone"]["altitude"];
    config.initialDir = configJson["drone"]["initialDirection"];
    config.attackSpeed = configJson["drone"]["attackSpeed"];
    config.accelPath = configJson["drone"]["accelerationPath"];
    config.angularSpeed = configJson["drone"]["angularSpeed"];
    config.turnThreshold = configJson["drone"]["turnThreshold"];
    config.ammoName = configJson["ammo"].get<std::string>();

    config.arrayTimeStep = configJson["targetArrayTimeStep"];
    config.simTimeStep = configJson["simulation"]["timeStep"];
    config.hitRadius = configJson["simulation"]["hitRadius"];

    return true;
}

bool JsonConfigLoader::loadAmmo(std::vector<AmmoParams>& ammoList) {
    std::ifstream ammoFile(ammoPath);
    if (!ammoFile) {
        ERROR_LOG("Failed to open file \"" << ammoPath << "\"");
        return false;
    }

    json ammoJson;
    ammoFile >> ammoJson;

    const int ammoCount = static_cast<int>(ammoJson.size());
    if (ammoCount <= 0) {
        ERROR_LOG("File \"" << ammoPath << "\" does not contain ammo entries");
        return false;
    }

    ammoList.clear();
    ammoList.reserve(ammoCount);

    for (int i = 0; i < ammoCount; ++i) {
        ammoList.emplace_back();
        AmmoParams& ammo = ammoList.back();
        ammo.name = ammoJson[i]["name"].get<std::string>();
        ammo.mass = ammoJson[i]["mass"];
        ammo.drag = ammoJson[i]["drag"];
        ammo.lift = ammoJson[i]["lift"];
    }

    return true;
}
