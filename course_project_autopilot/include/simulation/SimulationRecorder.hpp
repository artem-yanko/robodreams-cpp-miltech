#pragma once

#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"

#include <cstdint>
#include <string>
#include <vector>

struct SimulationStep {
    double positionX{};
    double positionY{};
    double direction{};
    int state{};
    int targetIndex{-1};
    double dropPointX{};
    double dropPointY{};
    double aimPointX{};
    double aimPointY{};
    double predictedTargetX{};
    double predictedTargetY{};
    double timeSecSinceStart{};
};

class SimulationRecorder {
public:
    explicit SimulationRecorder(std::string outputPath);

    void record(const MissionState& state, const MissionDecision& decision);
    bool write() const;
    const std::string& outputPath() const;

private:
    std::string outputFilePath;
    std::vector<SimulationStep> steps;
    uint32_t lastTelemetryMs{};
    bool hasLastTelemetry{};
};
