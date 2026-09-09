#pragma once

#include "domain/mission_decision.hpp"
#include "domain/types.hpp"
#include "utils/drone_link.hpp"

#include <memory>

class IDroneState;

struct WaypointNavigationOptions {
    bool stopAtGoal{};
    double arrivalRadius{};
};

struct WaypointNavigationResult {
    bool arrived{};
    double distanceToGoal{};
};

class WaypointNavigator {
public:
    WaypointNavigator();
    ~WaypointNavigator();

    WaypointNavigationResult update(
        const dlink::Telemetry& telemetry,
        DroneMotionState& droneMotion,
        const Coord& goal,
        const DroneConfig& config,
        double activeTurnThreshold,
        MissionDecision& decision,
        const WaypointNavigationOptions& options = {}
    );

    const char* stateName() const;
    bool isMoving() const;
    bool isDecelerating() const;
    bool isAccelerating() const;
    bool isTurning() const;

private:
    std::unique_ptr<IDroneState> state;
    bool stateBootstrapped{};
    bool stoppingAtGoal{};
};
