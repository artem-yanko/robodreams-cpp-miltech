#include "providers/JsonTargetProvider.hpp"

#include "utils/logger.hpp"
#include "utils/json.hpp"

#include <cmath>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>

static constexpr std::size_t kTargetHistoryLimit = 256;
using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const std::string& targetPath)
    : targetPath_(targetPath) {
}

void JsonTargetProvider::run(double targetTimeStep, double arrayTimeStep, double timeScale) {
    threadReady_ = true;
    DEBUG("Target provider thread ready");

    while (running_ && !started_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    DEBUG("Target provider thread started");

    const double normalizedTimeScale = timeScale > 0.0 ? timeScale : 1.0;
    auto lastTick = std::chrono::steady_clock::now();
    double accumulatedSimTime = 0.0;
    while (running_) {
        auto now = std::chrono::steady_clock::now();
        accumulatedSimTime += std::chrono::duration<double>(now - lastTick).count() * normalizedTimeScale;
        lastTick = now;

        while (running_
            && accumulatedSimTime >= targetTimeStep
            && currentTime_ + targetTimeStep <= allowedSimTime_) {
            step(targetTimeStep, arrayTimeStep);
            accumulatedSimTime -= targetTimeStep;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    DEBUG("Target provider thread stopped");
}

void JsonTargetProvider::start() {
    started_ = true;
    DEBUG("Target provider start signal received");
}

void JsonTargetProvider::stop() {
    running_ = false;
    DEBUG("Target provider stop signal received");
}

bool JsonTargetProvider::isThreadReady() const {
    return threadReady_;
}

double JsonTargetProvider::getCurrentSimTime() const {
    std::lock_guard<std::mutex> lock(currentTargetsMutex_);
    return currentTime_;
}

void JsonTargetProvider::setAllowedSimTime(double simTime) {
    allowedSimTime_ = simTime;
}

void JsonTargetProvider::clearTargets() {
    std::lock_guard<std::mutex> lock(currentTargetsMutex_);
    trajectoryData_.positions.clear();
    trajectoryData_.targetCount = 0;
    trajectoryData_.timeSteps = 0;
    currentTargets_.clear();
    currentTime_ = 0.0;
    targetHistory_.clear();
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
    targetHistory_.push_back({0.0, currentTargets_});
    DEBUG("Target provider loaded " << trajectoryData_.targetCount
        << " targets with " << trajectoryData_.timeSteps << " trajectory steps");
    return true;
}

int JsonTargetProvider::getTargetCount() const {
    std::lock_guard<std::mutex> lock(currentTargetsMutex_);
    return static_cast<int>(currentTargets_.size());
}

Target JsonTargetProvider::getTarget(int index) const {
    std::lock_guard<std::mutex> lock(currentTargetsMutex_);
    if (index < 0 || index >= static_cast<int>(currentTargets_.size())) {
        return {};
    }

    return currentTargets_[index];
}

Target JsonTargetProvider::getTargetAt(int index, double simTime) const {
    std::lock_guard<std::mutex> lock(currentTargetsMutex_);
    if (targetHistory_.empty()) {
        return {};
    }

    if (index < 0 || index >= static_cast<int>(targetHistory_.back().targets.size())) {
        return {};
    }

    if (simTime <= targetHistory_.front().simTime) {
        return targetHistory_.front().targets[index];
    }

    if (simTime >= targetHistory_.back().simTime) {
        return targetHistory_.back().targets[index];
    }

    for (std::size_t i = 1; i < targetHistory_.size(); ++i) {
        const TargetSnapshot& previous = targetHistory_[i - 1];
        const TargetSnapshot& current = targetHistory_[i];
        if (simTime > current.simTime) {
            continue;
        }

        double fraction = (simTime - previous.simTime) / (current.simTime - previous.simTime);
        Target result{};
        result.position = {
            previous.targets[index].position.x + (current.targets[index].position.x - previous.targets[index].position.x) * fraction,
            previous.targets[index].position.y + (current.targets[index].position.y - previous.targets[index].position.y) * fraction
        };
        result.velocity = {
            previous.targets[index].velocity.x + (current.targets[index].velocity.x - previous.targets[index].velocity.x) * fraction,
            previous.targets[index].velocity.y + (current.targets[index].velocity.y - previous.targets[index].velocity.y) * fraction
        };
        return result;
    }

    return targetHistory_.back().targets[index];
}

void JsonTargetProvider::step(double targetTimeStep, double arrayTimeStep) {
    if (trajectoryData_.timeSteps <= 0 || trajectoryData_.targetCount <= 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(currentTargetsMutex_);
    currentTime_ += targetTimeStep;
    updateCurrentTargets(targetTimeStep, arrayTimeStep);
    targetHistory_.push_back({currentTime_, currentTargets_});
    while (targetHistory_.size() > kTargetHistoryLimit) {
        targetHistory_.pop_front();
    }
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
