#pragma once

#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"
#include "domain/runtime_config.hpp"
#include <memory>

class IBallisticSolver;
class IDroneState;

class MissionProcessor {
public:
    MissionProcessor(const RuntimeConfig& config, std::unique_ptr<IBallisticSolver> solver);
    ~MissionProcessor();

    MissionDecision update(const MissionState& state);

private:
    const RuntimeConfig& config;
    std::unique_ptr<IBallisticSolver> solver;
    std::unique_ptr<IDroneState> droneState;
    DroneMotionState droneMotion;
    bool stateBootstrapped{};
    bool releasePhaseActive{};
    int releaseTargetIndex{-1};
    bool hasPreviousTelemetry{};
    dlink::Telemetry previousTelemetry{};
    uint32_t lastProcessedTelemetryMs{};
    bool hasPreviousReleaseTelemetry{};
    dlink::Telemetry previousReleaseTelemetry{};
    uint32_t lastReleaseCheckTelemetryMs{};
    double acceleration{};
    int lockedTargetIndex{-1};
    bool returningFromManuver{};
};
