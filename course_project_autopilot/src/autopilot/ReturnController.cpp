#include "autopilot/ReturnController.hpp"

#include "utils/math_utils.hpp"

#include <cmath>

namespace {

constexpr double RETURN_ARRIVAL_RADIUS = 3.0;

} // namespace

ReturnController::ReturnController(const DroneConfig& config, const Coord& destination)
    : config(config), destination(destination) {
}

ReturnControlResult ReturnController::update(const MissionState& state) {
    ReturnControlResult result{};
    if (!state.telemetryReceived) {
        return result;
    }

    const Coord position{state.telemetry.x, state.telemetry.y};
    const Coord direction = destination - position;
    result.distanceToDestination = distanceBetween(position, destination);
    droneMotion.currentDir = state.telemetry.dir;
    droneMotion.currentSpeed = state.telemetry.speed;
    droneMotion.desiredDir = result.distanceToDestination > 0.0
        ? std::atan2(direction.y, direction.x)
        : state.telemetry.dir;

    const WaypointNavigationResult navigation = navigator.update(
        state.telemetry,
        droneMotion,
        destination,
        config,
        config.turnThreshold,
        result.decision,
        WaypointNavigationOptions{
            .stopAtGoal = true,
            .arrivalRadius = RETURN_ARRIVAL_RADIUS
        }
    );

    result.arrived = navigation.arrived;
    result.distanceToDestination = navigation.distanceToGoal;
    return result;
}

const char* ReturnController::navigationStateName() const {
    return navigator.stateName();
}
