#include "core/MissionProcessor.hpp"
#include "core/TargetAnalyzer.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "utils/logger.hpp"
#include "utils/math_utils.hpp"

#include <cmath>

static bool isInsideRadius(const Coord& point, const Coord& center, double radius) {
    return distanceBetween(point, center) <= radius;
}

static double signedDistanceToReleaseBoundary(const Coord& point, const Coord& dropPoint, double releaseHeading) {
    Coord axis{std::cos(releaseHeading), std::sin(releaseHeading)};
    Coord delta = point - dropPoint;
    return delta.x * axis.x + delta.y * axis.y;
}

static double calculateReleasePredictionDt(const DroneConfig& config, uint32_t previousMs, uint32_t currentMs) {
    if (currentMs > previousMs) {
        return static_cast<double>(currentMs - previousMs) / 1000.0;
    }

    if (config.simTimeStep > 0.0) {
        return config.simTimeStep;
    }

    if (config.physicsTimeStep > 0.0) {
        return config.physicsTimeStep;
    }

    return 0.1;
}

static void clearLockedTarget(
    int& lockedTargetIndex,
    bool& releasePhaseActive,
    int& releaseTargetIndex,
    bool& hasPreviousReleaseTelemetry,
    uint32_t& lastReleaseCheckTelemetryMs,
    bool& returningFromManuver,
    DroneMotionState& droneMotion
) {
    lockedTargetIndex = -1;
    releasePhaseActive = false;
    releaseTargetIndex = -1;
    hasPreviousReleaseTelemetry = false;
    lastReleaseCheckTelemetryMs = 0;
    returningFromManuver = false;
    droneMotion.currentTargetIndex = -1;
}

MissionProcessor::MissionProcessor(const RuntimeConfig& config, std::unique_ptr<IBallisticSolver> solver)
    : config(config), solver(std::move(solver)) {
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
    const bool hasNewTelemetry = !hasPreviousTelemetry || state.telemetry.t_ms != lastProcessedTelemetryMs;

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
    const bool isTurning = navigator.isTurning();
    const bool isMoving = navigator.isMoving();
    const bool isDecelerating = navigator.isDecelerating();
    const bool isAccelerating = navigator.isAccelerating();

    bool targetSelected = false;
    if (lockedTargetIndex >= 0) {
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
            clearLockedTarget(
                lockedTargetIndex,
                releasePhaseActive,
                releaseTargetIndex,
                hasPreviousReleaseTelemetry,
                lastReleaseCheckTelemetryMs,
                returningFromManuver,
                droneMotion
            );
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

    if (lockedTargetIndex < 0) {
        lockedTargetIndex = best.targetIndex;
        LOG("TARGET lock acquired: target=" << lockedTargetIndex);
    }

    droneMotion.currentTargetIndex = best.targetIndex;

    if (releasePhaseActive && best.targetIndex != releaseTargetIndex) {
        LOG("RELEASE phase reset: current target=" << best.targetIndex
            << ", release target=" << releaseTargetIndex
            << ", reason=target changed");
        releasePhaseActive = false;
        releaseTargetIndex = -1;
        hasPreviousReleaseTelemetry = false;
        lastReleaseCheckTelemetryMs = 0;
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
        releasePhaseActive = false;
        releaseTargetIndex = -1;
        hasPreviousReleaseTelemetry = false;
        lastReleaseCheckTelemetryMs = 0;
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
    decision.impactPointX = dronePosition.x + std::cos(state.telemetry.dir) * ballistics.horizontalDistance;
    decision.impactPointY = dronePosition.y + std::sin(state.telemetry.dir) * ballistics.horizontalDistance;
    decision.impactDeltaX = decision.impactPointX - best.targetPos.x;
    decision.impactDeltaY = decision.impactPointY - best.targetPos.y;

    Coord goal{};
    bool headingToManeuver = best.needManeuver && !atManeuverPoint;
    if (headingToManeuver) {
        goal = best.maneuverPoint;
        releasePhaseActive = false;
        releaseTargetIndex = -1;
        hasPreviousReleaseTelemetry = false;
        lastReleaseCheckTelemetryMs = 0;
    } else {
        goal = best.dropPoint;
    }

    const double headingError = std::fabs(calculateAngleDifference(state.telemetry.dir, best.releaseHeading));
    const bool releaseReady =
        !headingToManeuver
        && lockedTargetIndex == best.targetIndex
        && navigator.isMoving()
        && std::fabs(state.telemetry.speed - config.drone.attackSpeed) <= 0.5
        && headingError <= best.releaseTurnThreshold;

    if (releaseReady) {
        if (!releasePhaseActive || releaseTargetIndex != best.targetIndex) {
            LOG("RELEASE phase entered: target=" << best.targetIndex
                << ", dropPoint=(" << best.dropPoint.x << ", " << best.dropPoint.y << ")"
                << ", releaseHeading=" << best.releaseHeading
                << ", headingError=" << headingError
                << ", speed=" << state.telemetry.speed);
        }
        releasePhaseActive = true;
        releaseTargetIndex = best.targetIndex;
        if (!hasPreviousReleaseTelemetry) {
            previousReleaseTelemetry = state.telemetry;
            hasPreviousReleaseTelemetry = true;
            lastReleaseCheckTelemetryMs = state.telemetry.t_ms;
        }
    }

    if (releasePhaseActive && releaseTargetIndex == best.targetIndex) {
        droneMotion.desiredDir = best.releaseHeading;
    } else {
        Coord deltaToGoal = goal - dronePosition;
        if (distanceBetween(dronePosition, goal) > 0.0) {
            droneMotion.desiredDir = std::atan2(deltaToGoal.y, deltaToGoal.x);
        } else {
            droneMotion.desiredDir = state.telemetry.dir;
        }
    }

    navigator.update(
        state.telemetry,
        droneMotion,
        goal,
        config.drone,
        config.drone.turnThreshold,
        decision
    );

    double angleError = calculateAngleDifference(state.telemetry.dir, droneMotion.desiredDir);
    decision.angleError = angleError;

    if (releasePhaseActive && !hasNewTelemetry) {
        LOG("RELEASE waiting: target=" << best.targetIndex
            << ", reason=no new telemetry"
            << ", dropPoint=(" << best.dropPoint.x << ", " << best.dropPoint.y << ")"
            << ", releaseHeading=" << best.releaseHeading);
    }

    const bool hasNewReleaseTelemetry =
        releasePhaseActive
        && releaseTargetIndex == best.targetIndex
        && state.telemetry.t_ms != lastReleaseCheckTelemetryMs;

    if (!state.dropDone
        && releasePhaseActive
        && releaseTargetIndex == best.targetIndex
        && ballistics.horizontalDistance > 0.0
        && hasPreviousReleaseTelemetry
        && hasNewReleaseTelemetry) {
        const bool movingForRelease = navigator.isMoving();
        const bool atAttackSpeedForRelease =
            std::fabs(state.telemetry.speed - config.drone.attackSpeed) <= 0.5;
        const bool headingOkForRelease = headingError <= best.releaseTurnThreshold;
        Coord previousPosition{previousReleaseTelemetry.x, previousReleaseTelemetry.y};
        double previousSignedDistance = signedDistanceToReleaseBoundary(previousPosition, best.dropPoint, best.releaseHeading);
        double currentSignedDistance = signedDistanceToReleaseBoundary(dronePosition, best.dropPoint, best.releaseHeading);
        const double predictionDt = calculateReleasePredictionDt(
            config.drone,
            previousReleaseTelemetry.t_ms,
            state.telemetry.t_ms
        );
        Coord predictedNextPosition{
            dronePosition.x + std::cos(state.telemetry.dir) * state.telemetry.speed * predictionDt,
            dronePosition.y + std::sin(state.telemetry.dir) * state.telemetry.speed * predictionDt
        };
        double nextSignedDistance = signedDistanceToReleaseBoundary(predictedNextPosition, best.dropPoint, best.releaseHeading);
        LOG("RELEASE check: target=" << best.targetIndex
            << ", state=" << navigator.stateName()
            << ", moving=" << movingForRelease
            << ", speed=" << state.telemetry.speed
            << ", attackSpeed=" << config.drone.attackSpeed
            << ", headingError=" << headingError
            << ", releaseTurnThreshold=" << best.releaseTurnThreshold
            << ", previousSignedDistance=" << previousSignedDistance
            << ", currentSignedDistance=" << currentSignedDistance
            << ", nextSignedDistance=" << nextSignedDistance);
        DEBUG("RELEASE boundary: target=" << best.targetIndex
              << ", previous=" << previousSignedDistance
              << ", current=" << currentSignedDistance
              << ", next=" << nextSignedDistance);
        if ((previousSignedDistance < 0.0 && currentSignedDistance >= 0.0)
            || (currentSignedDistance < 0.0 && nextSignedDistance >= 0.0)) {
            if (!movingForRelease || !atAttackSpeedForRelease || !headingOkForRelease) {
                LOG("RELEASE boundary crossed with soft guard miss: target=" << best.targetIndex
                    << ", moving=" << movingForRelease
                    << ", atAttackSpeed=" << atAttackSpeedForRelease
                    << ", headingOk=" << headingOkForRelease);
            }
            if (currentSignedDistance < 0.0 && nextSignedDistance >= 0.0) {
                LOG("RELEASE predictive trigger: target=" << best.targetIndex
                    << ", currentSignedDistance=" << currentSignedDistance
                    << ", nextSignedDistance=" << nextSignedDistance
                    << ", predictionDt=" << predictionDt);
            }
            decision.shouldDrop = true;
        } else if (currentSignedDistance >= 0.0) {
            LOG("TARGET abort: target=" << best.targetIndex
                << ", reason=passed release boundary without drop");
            clearLockedTarget(
                lockedTargetIndex,
                releasePhaseActive,
                releaseTargetIndex,
                hasPreviousReleaseTelemetry,
                lastReleaseCheckTelemetryMs,
                returningFromManuver,
                droneMotion
            );
        }
    }

    if (hasNewReleaseTelemetry) {
        previousReleaseTelemetry = state.telemetry;
        hasPreviousReleaseTelemetry = true;
        lastReleaseCheckTelemetryMs = state.telemetry.t_ms;
    }

    if (hasNewTelemetry) {
        previousTelemetry = state.telemetry;
        hasPreviousTelemetry = true;
        lastProcessedTelemetryMs = state.telemetry.t_ms;
    }

    return decision;
}
