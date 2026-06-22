#include "providers/JsonTargetProvider.hpp"
#include "utils/logger.hpp"
#include "utils/json.hpp"
#include <cmath>
#include <fstream>
#include <string>
using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const std::string& targetPath)
    : targetPath_(targetPath) {
}

void JsonTargetProvider::clearTargets() {
  targets.positions.clear();
  targets.targetCount = 0;
  targets.timeSteps = 0;
}

JsonTargetProvider::~JsonTargetProvider() {
  clearTargets();
}

bool JsonTargetProvider::loadTargets() {
  std::ifstream targetsFile(targetPath_);
  if (!targetsFile) {
    ERROR_LOG("Failed to open file \"" << targetPath_ << "\"");
    return false;
  }

  json targetsJson;
  targetsFile >> targetsJson;

  clearTargets();

  targets.targetCount = targetsJson["targetCount"];
  targets.timeSteps = targetsJson["timeSteps"];
  if (targets.targetCount <= 0 || targets.timeSteps <= 0) {
    ERROR_LOG("File \"" << targetPath_ << "\" contains invalid target arrays");
    return false;
  }

  targets.positions.resize(targets.targetCount);
  for (int i = 0; i < targets.targetCount; ++i) {
    targets.positions[i].resize(targets.timeSteps);
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
