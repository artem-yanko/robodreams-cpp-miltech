#include "providers/JsonConfigLoader.hpp"
#include "utils/logger.hpp"
#include <fstream>
#include <string>
#include "utils/json.hpp"
using json = nlohmann::json;

JsonConfigLoader::JsonConfigLoader(const std::string& configPath, const std::string& ammoPath)
    : configPath_(configPath), ammoPath_(ammoPath) {
}

bool JsonConfigLoader::loadConfig(DroneConfig& config) {
    std::ifstream configFile(configPath_);
    if (!configFile) {
        ERROR_LOG("Failed to open config file \"" << configPath_ << "\"");
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

bool JsonConfigLoader::loadAmmo(AmmoParams*& ammoList, int& ammoCount) {
  std::ifstream ammoFile(ammoPath_);
  if (!ammoFile) {
    ERROR_LOG("Failed to open file \"" << ammoPath_ << "\"");
    return false;
  }

  json ammoJson;
  ammoFile >> ammoJson;

  ammoCount = static_cast<int>(ammoJson.size());
  if (ammoCount <= 0) {
    ERROR_LOG("File \"" << ammoPath_ << "\" does not contain ammo entries");
    return false;
  }

  ammoList = new AmmoParams[ammoCount]{};
  for (int i = 0; i < ammoCount; ++i) {
    ammoList[i].name = ammoJson[i]["name"].get<std::string>();
    ammoList[i].mass = ammoJson[i]["mass"];
    ammoList[i].drag = ammoJson[i]["drag"];
    ammoList[i].lift = ammoJson[i]["lift"];
  }

  return true;
}
