#pragma once

#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"
#include "domain/runtime_config.hpp"

class MissionProcessor {
public:
    explicit MissionProcessor(const RuntimeConfig& config);

    MissionDecision update(const MissionState& state) const;

private:
    const RuntimeConfig& config;
};
