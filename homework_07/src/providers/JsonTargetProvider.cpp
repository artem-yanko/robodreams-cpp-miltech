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
    trajectoryData_.positions.clear();
    trajectoryData_.targetCount = 0;
    trajectoryData_.timeSteps = 0;
    currentTargets_.clear();
    currentTime_ = 0.0;
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
    for (int i = 0; i < trajectoryData_.targetCount; ++i) {
        currentTargets_[i].position = trajectoryData_.positions[i].front();
        currentTargets_[i].velocity = {};
    }
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

void JsonTargetProvider::step(double targetTimeStep, double arrayTimeStep) {
    if (trajectoryData_.timeSteps <= 0 || trajectoryData_.targetCount <= 0) {
        return;
    }

    currentTime_ += targetTimeStep;
    updateCurrentTargets(targetTimeStep, arrayTimeStep);
}

Coord JsonTargetProvider::interpolateTargetPosition(int targetIndex, double time, double arrayTimeStep) const {
    if (targetIndex < 0 || targetIndex >= trajectoryData_.targetCount || trajectoryData_.timeSteps <= 0) {
        return {};
    }

    if (arrayTimeStep <= 0.0) {
        return trajectoryData_.positions[targetIndex].front();
    }

    double targetTime = time / arrayTimeStep;
    int baseIdx = static_cast<int>(std::floor(targetTime));
    int idx = baseIdx % trajectoryData_.timeSteps;
    if (idx < 0) {
        idx += trajectoryData_.timeSteps;
    }
    int next = (idx + 1) % trajectoryData_.timeSteps;
    double frac = targetTime - baseIdx;

    return trajectoryData_.positions[targetIndex][idx] +
           (trajectoryData_.positions[targetIndex][next] - trajectoryData_.positions[targetIndex][idx]) * frac;
}

void JsonTargetProvider::updateCurrentTargets(double targetTimeStep, double arrayTimeStep) {
    if (trajectoryData_.targetCount <= 0 || trajectoryData_.timeSteps <= 0) {
        return;
    }

    for (int i = 0; i < trajectoryData_.targetCount; ++i) {
        Target& target = currentTargets_[i];
        Coord previousPosition = target.position;
        target.position = interpolateTargetPosition(i, currentTime_, arrayTimeStep);

        if (targetTimeStep <= 0.0) {
            target.velocity = {};
            continue;
        }

        target.velocity = (target.position - previousPosition) / targetTimeStep;
    }
}
