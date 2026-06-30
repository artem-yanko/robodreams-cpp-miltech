#include "core/MissionProcessor.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateAccelerating.hpp"
#include "states/StateDecelerating.hpp"
#include "states/StateMoving.hpp"
#include "states/StateStopped.hpp"
#include "states/StateTurning.hpp"
#include "utils/math_utils.hpp"

#include <cmath>
#include <cstring>

static const double SPEED_EPSILON = 1e-6;
static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;

static std::unique_ptr<IDroneState> bootstrapStateFromTelemetry(
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

MissionProcessor::MissionProcessor(const RuntimeConfig& config, std::unique_ptr<IBallisticSolver> solver)
    : config(config), solver(std::move(solver)), droneState(std::make_unique<StateStopped>()) {
    droneMotion.currentDir = config.drone.initialDir;
}

MissionProcessor::~MissionProcessor() = default;

MissionDecision MissionProcessor::update(const MissionState& state) {
    MissionDecision decision{};

    if (!state.telemetryReceived || state.targets.empty()) {
        return decision;
    }

    const dlink::TargetPos& target = state.targets.front();
    decision.targetId = static_cast<int>(target.id);
    Coord dronePosition{state.telemetry.x, state.telemetry.y};
    Coord targetPosition{target.x, target.y};
    Coord deltaToTarget = targetPosition - dronePosition;
    const double distanceToTarget = distanceBetween(dronePosition, targetPosition);
    decision.distanceToTarget = distanceToTarget;

    droneMotion.currentDir = state.telemetry.dir;
    droneMotion.currentSpeed = state.telemetry.speed;

    AmmoParams ammo{};
    if (state.ammoReceived) {
        ammo.name = state.ammo.name;
        ammo.mass = state.ammo.mass;
        ammo.drag = state.ammo.drag;
        ammo.lift = state.ammo.lift;
    } else {
        ammo.name = config.drone.ammoName;
    }

    BallisticsResult ballistics{};
    if (solver != nullptr && state.ammoReceived) {
        ballistics = solver->solve(config.drone, ammo);
    }

    double dropPointX = targetPosition.x;
    double dropPointY = targetPosition.y;
    double distanceToDropPoint = distanceToTarget;
    if (ballistics.horizontalDistance > 0.0 && distanceToTarget > 0.0) {
        Coord directionToTarget = normalize(deltaToTarget);
        Coord dropPoint = targetPosition - directionToTarget * ballistics.horizontalDistance;
        dropPointX = dropPoint.x;
        dropPointY = dropPoint.y;
        distanceToDropPoint = distanceBetween(dronePosition, dropPoint);
    }

    decision.dropPointX = dropPointX;
    decision.dropPointY = dropPointY;
    decision.distanceToDropPoint = distanceToDropPoint;

    Coord goal{dropPointX, dropPointY};
    Coord deltaToGoal = goal - dronePosition;
    if (distanceBetween(dronePosition, goal) > 0.0) {
        droneMotion.desiredDir = std::atan2(deltaToGoal.y, deltaToGoal.x);
    } else {
        droneMotion.desiredDir = state.telemetry.dir;
    }

    if (!stateBootstrapped) {
        droneState = bootstrapStateFromTelemetry(state.telemetry, droneMotion, config.drone);
        if (std::strcmp(droneState->name(), "Turning") == 0) {
            droneMotion.turnTargetDir = droneMotion.desiredDir;
        }
        stateBootstrapped = true;
    }

    DroneContext ctx{
        .telemetry = state.telemetry,
        .droneMotion = droneMotion,
        .goal = goal,
        .config = config.drone,
        .activeTurnThreshold = config.drone.turnThreshold,
        .decision = decision
    };

    std::unique_ptr<IDroneState> nextState = droneState->execute(ctx);
    if (nextState) {
        droneState = std::move(nextState);
    }

    double angleError = calculateAngleDifference(state.telemetry.dir, droneMotion.desiredDir);
    decision.angleError = angleError;
    const double absAngleError = std::fabs(angleError);

    if (!state.dropDone
        && ballistics.horizontalDistance > 0.0
        && distanceToDropPoint <= config.drone.hitRadius
        && droneState->isMoving()
        && std::fabs(state.telemetry.speed - config.drone.attackSpeed) <= 0.5
        && absAngleError <= config.drone.turnThreshold) {
        decision.shouldDrop = true;
    }

    return decision;
}
