#pragma once

#include "core/WaypointNavigator.hpp"
#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"
#include "domain/types.hpp"

struct ReturnControlResult {
    MissionDecision decision{};
    bool arrived{};
    double distanceToDestination{};
};

class ReturnController {
public:
    ReturnController(const DroneConfig& config, const Coord& destination);

    ReturnControlResult update(const MissionState& state);
    const char* navigationStateName() const;

private:
    DroneConfig config;
    Coord destination;
    WaypointNavigator navigator;
    DroneMotionState droneMotion{};
};
