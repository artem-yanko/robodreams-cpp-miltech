#include "core/WaypointNavigator.hpp"

#include "interfaces/IDroneState.hpp"
#include "states/StateAccelerating.hpp"
#include "states/StateDecelerating.hpp"
#include "states/StateMoving.hpp"
#include "states/StateStopped.hpp"
#include "states/StateTurning.hpp"
#include "utils/math_utils.hpp"

#include <cmath>
#include <utility>

namespace {

constexpr double SPEED_EPSILON = 1e-6;
constexpr double SLOW_TURN_THRESHOLD_FACTOR = 3.0;

std::unique_ptr<IDroneState> bootstrapStateFromTelemetry(
    const dlink::Telemetry& telemetry,
    const DroneMotionState& droneMotion,
    const DroneConfig& config
) {
    const double angleLeft = std::fabs(calculateAngleDifference(telemetry.dir, droneMotion.desiredDir));
    const double slowTurnThreshold = config.turnThreshold * SLOW_TURN_THRESHOLD_FACTOR;

    if (telemetry.speed <= SPEED_EPSILON) {
        if (angleLeft > config.turnThreshold) {
            return std::make_unique<StateTurning>();
        }
        return std::make_unique<StateStopped>();
    }

    if (angleLeft > slowTurnThreshold) {
        return std::make_unique<StateDecelerating>();
    }

    if (telemetry.speed >= config.attackSpeed - SPEED_EPSILON) {
        return std::make_unique<StateMoving>();
    }

    return std::make_unique<StateAccelerating>();
}

} // namespace

WaypointNavigator::WaypointNavigator()
    : state(std::make_unique<StateStopped>()) {
}

WaypointNavigator::~WaypointNavigator() = default;

void WaypointNavigator::update(
    const dlink::Telemetry& telemetry,
    DroneMotionState& droneMotion,
    const Coord& goal,
    const DroneConfig& config,
    double activeTurnThreshold,
    MissionDecision& decision
) {
    if (!stateBootstrapped) {
        state = bootstrapStateFromTelemetry(telemetry, droneMotion, config);
        if (state->isTurning()) {
            droneMotion.turnTargetDir = droneMotion.desiredDir;
        }
        stateBootstrapped = true;
    }

    DroneContext context{
        .telemetry = telemetry,
        .droneMotion = droneMotion,
        .goal = goal,
        .config = config,
        .activeTurnThreshold = activeTurnThreshold,
        .decision = decision
    };

    std::unique_ptr<IDroneState> nextState = state->execute(context);
    if (nextState) {
        state = std::move(nextState);
    }
}

const char* WaypointNavigator::stateName() const {
    return state ? state->name() : "Unknown";
}

bool WaypointNavigator::isMoving() const {
    return state && state->isMoving();
}

bool WaypointNavigator::isDecelerating() const {
    return state && state->isDecelerating();
}

bool WaypointNavigator::isAccelerating() const {
    return state && state->isAccelerating();
}

bool WaypointNavigator::isTurning() const {
    return state && state->isTurning();
}
