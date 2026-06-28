#pragma once

#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"
#include "domain/runtime_config.hpp"
#include <memory>

class IBallisticSolver;

class MissionProcessor {
public:
    MissionProcessor(const RuntimeConfig& config, std::unique_ptr<IBallisticSolver> solver);

    MissionDecision update(const MissionState& state) const;

private:
    const RuntimeConfig& config;
    std::unique_ptr<IBallisticSolver> solver;
};
