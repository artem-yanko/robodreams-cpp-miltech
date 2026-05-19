#include <fstream>
#include <iostream>
#include <cstring>
#include "json.hpp"
using json = nlohmann::json;

struct Coord {
    double x{};
    double y{};
};

struct TargetData {
  int targetCount{};
  int timeSteps{};
  Coord** positions{};
};

struct AmmoParams {
  char name[32]{};
  double mass{};
  double drag{};
  double lift{};
};

struct DroneConfig {
  Coord startPos{};
  double altitude{};
  double initialDir{};
  double attackSpeed{};
  double accelPath{};
  char ammoName[32]{};
  double arrayTimeStep{};
  double simTimeStep{};
  double hitRadius{};
  double angularSpeed{};
  double turnThreshold{};
};

bool readTargets(double targetXInTime[5][60], double targetYInTime[5][60]) {

    std::ifstream targetsFile("targets.txt");
    if (!targetsFile) {
        std::cerr << "ERROR: Не вдалося відкрити файл \"targets.txt\"" << std::endl;
        return false;
    }

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 60; ++j) {
            if (!(targetsFile >> targetXInTime[i][j])) {
                return false;
            }
        }
    }

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 60; ++j) {
            if (!(targetsFile >> targetYInTime[i][j])) {
                return false;
            }
        }
    }
    return true;

}

bool readTargetsJson(){
    std::ifstream fin("config.json");
    json j;
    fin >> j;

    TargetData targets;

    return true;
}

bool readConfigJson(DroneConfig& config){
    std::ifstream fin("config.json");
    json j;
    fin >> j;

    config.startPos.x = j["drone"]["position"]["x"];
    config.altitude = j["drone"]["altitude"];
    config.initialDir = j["drone"]["initialDirection"];

    return true;
}

bool readAmmoJson(AmmoParams*& ammoList, int& ammoCount) {
std::ifstream ammoFile("ammo.json");
if (!ammoFile) {
    return false;
}

json ammoJson; ammoFile >> ammoJson;

ammoCount = ammoJson.size();
AmmoParams* ammo = new AmmoParams[ammoCount];

for (int i = 0; i < ammoCount; ++i) {
    std::string name = ammoJson[i]["name"];
    std::strncpy(ammoList[i].name, name.c_str(), sizeof(ammoList[i].name) - 1);
    ammoList[i].name[sizeof(ammoList[i].name) - 1] = '\0';

    ammoList[i].mass = ammoJson[i]["mass"];
    ammoList[i].drag = ammoJson[i]["drag"];
    ammoList[i].lift = ammoJson[i]["lift"];
}

return true;
}

const AmmoParams* ammoSelect(const AmmoParams* ammoList, int ammoCount, const char* ammoName) {
for (int i = 0; i < ammoCount; ++i) {
    if (std::strcmp(ammoName, ammoList[i].name) == 0) {
    return &ammoList[i];
    }
}

return nullptr;
}

int main() {

    // config reading test
    DroneConfig config;

    readConfigJson(config);

    std::cout << config.altitude << std::endl;


    return 0;
}