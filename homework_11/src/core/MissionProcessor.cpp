#include "core/MissionProcessor.hpp"
#include "core/TargetAnalyzer.hpp"
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
static const double TARGET_SWITCH_PREVENTION = 1.0;

static double calculateDroneAcceleration(double attackSpeed, double accelerationPath) {
    if (accelerationPath <= 0.0) {
        return 0.0;
    }
    return attackSpeed * attackSpeed / (2.0 * accelerationPath);
}

static bool isInsideRadius(const Coord& point, const Coord& center, double radius) {
    return distanceBetween(point, center) <= radius;
}

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
    acceleration = calculateDroneAcceleration(config.drone.attackSpeed, config.drone.accelPath);
}

MissionProcessor::~MissionProcessor() = default;

MissionDecision MissionProcessor::update(const MissionState& state) {
    MissionDecision decision{};
    TargetAnalyzer analyzer{};

    if (!state.telemetryReceived || state.targets.empty()) {
        return decision;
    }

    Coord dronePosition{state.telemetry.x, state.telemetry.y};
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

    BestTargetResult best{};
    const bool isTurning = droneState->isTurning();
    const bool isMoving = droneState->isMoving();
    const bool isDecelerating = droneState->isDecelerating();
    const bool isAccelerating = droneState->isAccelerating();

    bool targetSelected = false;
    if (targetLocked && lockedTargetIndex >= 0) {
        targetSelected = analyzer.evaluateTarget(
            best,
            state,
            config.drone,
            droneMotion,
            isTurning,
            isMoving,
            isDecelerating,
            isAccelerating,
            static_cast<std::size_t>(lockedTargetIndex),
            dronePosition,
            ballistics,
            acceleration,
            returningFromManuver
        );

        if (!targetSelected) {
            targetLocked = false;
            lockedTargetIndex = -1;
            candidateTargetIndex = -1;
            candidateTargetStreak = 0;
            returningFromManuver = false;
            maneuverTargetIndex = -1;
            droneMotion.currentTargetIndex = -1;
        }
    }

    if (!targetSelected) {
        targetSelected = analyzer.selectBestTarget(
            best,
            state,
            config.drone,
            droneMotion,
            isTurning,
            isMoving,
            isDecelerating,
            isAccelerating,
            dronePosition,
            ballistics,
            acceleration,
            returningFromManuver
        );
    }

    if (!targetSelected) {
        return decision;
    }

    droneMotion.currentTargetIndex = best.targetIndex;

    if (!targetLocked) {
        if (best.targetIndex == candidateTargetIndex) {
            ++candidateTargetStreak;
        } else {
            candidateTargetIndex = best.targetIndex;
            candidateTargetStreak = 1;
        }
    } else {
        candidateTargetIndex = lockedTargetIndex;
        candidateTargetStreak = 0;
    }

    if (best.targetIndex != maneuverTargetIndex) {
        returningFromManuver = false;
    }
    if (isMoving && !best.needManeuver) {
        returningFromManuver = true;
    }
    if (returningFromManuver) {
        best.needManeuver = false;
    }

    bool atManeuverPoint = best.needManeuver && isInsideRadius(dronePosition, best.maneuverPoint, config.drone.hitRadius);
    if (atManeuverPoint) {
        returningFromManuver = true;
        best.needManeuver = false;
        targetLocked = true;
        lockedTargetIndex = best.targetIndex;
        candidateTargetIndex = best.targetIndex;
        candidateTargetStreak = 0;
    }

    decision.targetId = best.targetIndex;
    decision.distanceToTarget = distanceBetween(dronePosition, best.targetPos);
    decision.predictedTargetX = best.predictedTarget.x;
    decision.predictedTargetY = best.predictedTarget.y;
    decision.dropPointX = best.dropPoint.x;
    decision.dropPointY = best.dropPoint.y;
    decision.releaseHeading = best.releaseHeading;
    decision.releaseTurnThreshold = best.releaseTurnThreshold;
    const double distanceToDropPoint = distanceBetween(dronePosition, best.dropPoint);
    decision.distanceToDropPoint = distanceToDropPoint;

    Coord goal{};
    bool headingToManeuver = best.needManeuver && !atManeuverPoint;
    if (headingToManeuver) {
        goal = best.maneuverPoint;
        maneuverTargetIndex = best.targetIndex;
        targetLocked = true;
        lockedTargetIndex = best.targetIndex;
        candidateTargetIndex = best.targetIndex;
        candidateTargetStreak = 0;
    } else {
        goal = best.dropPoint;
        if (!targetLocked && candidateTargetStreak >= 2) {
            targetLocked = true;
            lockedTargetIndex = best.targetIndex;
            candidateTargetStreak = 0;
        }
    }

    if (!headingToManeuver && isMoving && !targetLocked && distanceToDropPoint <= ballistics.horizontalDistance + TARGET_SWITCH_PREVENTION) {
        targetLocked = true;
        lockedTargetIndex = best.targetIndex;
        candidateTargetIndex = best.targetIndex;
        candidateTargetStreak = 0;
    }

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
    const double headingError = std::fabs(calculateAngleDifference(state.telemetry.dir, best.releaseHeading));

    if (!state.dropDone
        && ballistics.horizontalDistance > 0.0
        && !headingToManeuver
        && distanceToDropPoint <= config.drone.hitRadius
        && droneState->isMoving()
        && std::fabs(state.telemetry.speed - config.drone.attackSpeed) <= 0.5
        && headingError <= best.releaseTurnThreshold) {
        decision.shouldDrop = true;
    }

    return decision;
}
