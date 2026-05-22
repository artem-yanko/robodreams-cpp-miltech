#include "providers/JsonConfigLoader.hpp"
#include "utils/logger.hpp"
#include <cstring>
#include <fstream>
#include <string>
#include "utils/json.hpp"
using json = nlohmann::json;

bool JsonConfigLoader::loadConfig(DroneConfig& config) {
    std::ifstream configFile("config.json");
    if (!configFile) {
        ERROR_LOG("Не вдалося відкрити файл конфігурації");
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

    std::strncpy(config.ammoName, configJson["ammo"].get<std::string>().c_str(), sizeof(config.ammoName) - 1);
    config.ammoName[sizeof(config.ammoName) - 1] = '\0';

    config.arrayTimeStep = configJson["targetArrayTimeStep"];
    config.simTimeStep = configJson["simulation"]["timeStep"];
    config.hitRadius = configJson["simulation"]["hitRadius"];

    return true;
}

bool JsonConfigLoader::loadAmmo(AmmoParams*& ammoList, int& ammoCount) {
  std::ifstream ammoFile("ammo.json");
  if (!ammoFile) {
    ERROR_LOG("Не вдалося відкрити файл \"ammo.json\"");
    return false;
  }

  json ammoJson;
  ammoFile >> ammoJson;

  ammoCount = static_cast<int>(ammoJson.size());
  if (ammoCount <= 0) {
    ERROR_LOG("Файл \"ammo.json\" не містить боєприпасів");
    return false;
  }

  ammoList = new AmmoParams[ammoCount]{};
  for (int i = 0; i < ammoCount; ++i) {
    std::strncpy(ammoList[i].name, ammoJson[i]["name"].get<std::string>().c_str(), sizeof(ammoList[i].name) - 1);
    ammoList[i].name[sizeof(ammoList[i].name) - 1] = '\0';

    ammoList[i].mass = ammoJson[i]["mass"];
    ammoList[i].drag = ammoJson[i]["drag"];
    ammoList[i].lift = ammoJson[i]["lift"];
  }

  return true;
}
