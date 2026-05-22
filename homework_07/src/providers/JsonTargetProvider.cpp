#include "providers/JsonTargetProvider.hpp"
#include "utils/logger.hpp"
#include "utils/json.hpp"
#include <cmath>
#include <fstream>
#include <string>
using json = nlohmann::json;

void JsonTargetProvider::clearTargets() {
  if (targets.positions) {
    for (int i = 0; i < targets.targetCount; ++i) {
      delete[] targets.positions[i];
    }
    delete[] targets.positions;
    targets.positions = nullptr;
  }
  targets.targetCount = 0;
  targets.timeSteps = 0;
}

JsonTargetProvider::~JsonTargetProvider() {
  clearTargets();
}

bool JsonTargetProvider::loadTargets() {
  std::ifstream targetsFile("targets.json");
  if (!targetsFile) {
    ERROR_LOG("Не вдалося відкрити файл \"targets.json\"");
    return false;
  }

  json targetsJson;
  targetsFile >> targetsJson;

  clearTargets();

  targets.targetCount = targetsJson["targetCount"];
  targets.timeSteps = targetsJson["timeSteps"];
  if (targets.targetCount <= 0 || targets.timeSteps <= 0) {
    ERROR_LOG("Файл \"targets.json\" містить некоректні розміри масиву цілей");
    return false;
  }

  targets.positions = new Coord*[targets.targetCount]{};
  for (int i = 0; i < targets.targetCount; ++i) {
    targets.positions[i] = new Coord[targets.timeSteps]{};
    for (int j = 0; j < targets.timeSteps; ++j) {
      targets.positions[i][j].x = targetsJson["targets"][i]["positions"][j]["x"];
      targets.positions[i][j].y = targetsJson["targets"][i]["positions"][j]["y"];
    }
  }

  return true;
}

int JsonTargetProvider::getTargetCount() {
  return targets.targetCount;
}

const TargetData& JsonTargetProvider::getTargetsData() {
  return targets;
}
