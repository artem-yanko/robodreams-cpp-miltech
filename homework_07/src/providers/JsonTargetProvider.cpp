#include "providers/JsonTargetProvider.hpp"
#include "utils/logger.hpp"
#include "utils/json.hpp"
#include <cmath>
#include <fstream>
#include <string>
using json = nlohmann::json;

static bool interpolateTargetPosition(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep, Coord& targetPos) {
  if (arrayTimeStep == 0.0) {
    ERROR_LOG("Крок часу масиву цілей не може дорівнювати нулю!");
    return false;
  }
  if (targetIndex < 0 || targetIndex >= targets.targetCount) {
    ERROR_LOG("Невірний індекс цілі: " << targetIndex);
    return false;
  }
  if (simulationTime < 0) {
    ERROR_LOG("Час не може бути від'ємним!");
    return false;
  }


  double targetTime = simulationTime / arrayTimeStep;
  int baseIdx = (int)floor(targetTime);
  int idx = baseIdx % targets.timeSteps;
  int next = (idx + 1) % targets.timeSteps;
  double frac = targetTime - baseIdx;
  targetPos = targets.positions[targetIndex][idx] + (targets.positions[targetIndex][next] - targets.positions[targetIndex][idx]) * frac;
  return true;
}

static bool calculateTargetVelocity(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep, Coord& velocity) {
  if (arrayTimeStep == 0.0) {
    ERROR_LOG("Крок часу масиву цілей не може дорівнювати нулю!");
    return false;
  }

  double dt = arrayTimeStep;

  Coord firstPos{};
  if (!interpolateTargetPosition(targetIndex, targets, simulationTime, arrayTimeStep, firstPos)) {
    return false;
  }

  Coord secondPos{};
  if (!interpolateTargetPosition(targetIndex, targets, simulationTime + dt, arrayTimeStep, secondPos)) {
    return false;
  }

  velocity = (secondPos - firstPos) / dt;

  return true;
}

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

bool JsonTargetProvider::loadTargets(double arrayTimeStep) {
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

  this->arrayTimeStep = arrayTimeStep;
  simulationTime = 0.0;

  return true;
}

void JsonTargetProvider::setSimulationTime(double time) {
    simulationTime = time;
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
  if (!interpolateTargetPosition(index, targets, simulationTime, arrayTimeStep, target.position)) {
      ERROR_LOG("Не вдалося інтерполювати позицію цілі");
      return {};
  }

  if (!calculateTargetVelocity(index, targets, simulationTime, arrayTimeStep, target.velocity)) {
      ERROR_LOG("Не вдалося розрахувати швидкість цілі");
      return {};
  }
  return target;
}
