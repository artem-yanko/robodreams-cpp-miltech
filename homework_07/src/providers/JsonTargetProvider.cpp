#include "providers/JsonTargetProvider.hpp"
#include "utils/json.hpp"
#include <fstream>
#include "utils/logger.hpp"
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

Target JsonTargetProvider::getTarget(int index) {
  if (index < 0 || index >= targets.targetCount) {
    ERROR_LOG("Невірний індекс цілі: " << index);
    return {};
  }
  Target target{};
  target.position = targets.positions[index][0];
  target.velocity = {};
  return target;
}

