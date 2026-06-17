#include "providers/JsonTargetProvider.hpp"

#include "utils/logger.hpp"
#include "utils/json.hpp"

#include <fstream>
#include <string>

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const std::string& targetPath)
    : targetPath_(targetPath) {
}

void JsonTargetProvider::clearTargets() {
    trajectoryData_.positions.clear();
    trajectoryData_.targetCount = 0;
    trajectoryData_.timeSteps = 0;
    currentTargets_.clear();
    currentStep_ = 0;
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

    trajectoryData_.targetCount = targetsJson["targetCount"];
    trajectoryData_.timeSteps = targetsJson["timeSteps"];
    if (trajectoryData_.targetCount <= 0 || trajectoryData_.timeSteps <= 0) {
        ERROR_LOG("File \"" << targetPath_ << "\" contains invalid target arrays");
        return false;
    }

    trajectoryData_.positions.resize(trajectoryData_.targetCount);
    for (int i = 0; i < trajectoryData_.targetCount; ++i) {
        trajectoryData_.positions[i].resize(trajectoryData_.timeSteps);
        for (int j = 0; j < trajectoryData_.timeSteps; ++j) {
            trajectoryData_.positions[i][j].x = targetsJson["targets"][i]["positions"][j]["x"];
            trajectoryData_.positions[i][j].y = targetsJson["targets"][i]["positions"][j]["y"];
        }
    }

    currentTargets_.resize(trajectoryData_.targetCount);
    updateCurrentTargets(0.0);
    return true;
}

int JsonTargetProvider::getTargetCount() const {
    return trajectoryData_.targetCount;
}

Target JsonTargetProvider::getTarget(int index) const {
    if (index < 0 || index >= static_cast<int>(currentTargets_.size())) {
        return {};
    }

    return currentTargets_[index];
}

void JsonTargetProvider::step(double targetTimeStep) {
    if (trajectoryData_.timeSteps <= 0 || trajectoryData_.targetCount <= 0) {
        return;
    }

    if (currentStep_ + 1 < trajectoryData_.timeSteps) {
        ++currentStep_;
    }

    updateCurrentTargets(targetTimeStep);
}

void JsonTargetProvider::updateCurrentTargets(double targetTimeStep) {
    if (trajectoryData_.targetCount <= 0 || trajectoryData_.timeSteps <= 0) {
        return;
    }

    for (int i = 0; i < trajectoryData_.targetCount; ++i) {
        Target& target = currentTargets_[i];
        target.position = trajectoryData_.positions[i][currentStep_];

        if (targetTimeStep <= 0.0 || currentStep_ + 1 >= trajectoryData_.timeSteps) {
            target.velocity = {};
            continue;
        }

        Coord nextPosition = trajectoryData_.positions[i][currentStep_ + 1];
        target.velocity = (nextPosition - target.position) / targetTimeStep;
    }
}
