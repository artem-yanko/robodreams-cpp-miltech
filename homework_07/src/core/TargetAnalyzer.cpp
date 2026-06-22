#include "core/TargetAnalyzer.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "utils/logger.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

static const double VERY_LARGE_TIME = 1e18;
static const int TARGET_PREDICTION_ITERATIONS = 3;
static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;

static double calculateTimeToStop(bool isTurning, double currentSpeed, double acceleration, double turnRemainingTime) {
    if (isTurning) {
        return turnRemainingTime;
    }

    if (acceleration <= 0.0 || currentSpeed <= 0.0) {
        return 0.0;
    }

    return currentSpeed / acceleration;
}

static double estimateTravelTime(double distance, double startSpeed, double attackSpeed, double acceleration) {
    if (distance <= 0.0) {
        return 0.0;
    }

    if (attackSpeed <= 0.0) {
      return 0.0;
    }

    if (acceleration <= 0.0) {
        if (startSpeed > 0.0) {
            return distance / startSpeed;
        }
        return distance / attackSpeed;
    }

    if (startSpeed >= attackSpeed) {
        return distance / startSpeed;
    }

    double timeToFullSpeed = (attackSpeed - startSpeed) / acceleration;
    double distanceToFullSpeed = (startSpeed + attackSpeed) * timeToFullSpeed / 2.0;
    // Якщо закоротка дистанція для розгону до attackSpeed
    if (distance <= distanceToFullSpeed) {
        return (-startSpeed + sqrt(startSpeed * startSpeed + 2.0 * acceleration * distance)) / acceleration;
    }

    return timeToFullSpeed + (distance - distanceToFullSpeed) / attackSpeed;
}

static double distanceToFullSpeed(double startSpeed, double attackSpeed, double acceleration) {
    if (attackSpeed <= 0.0 || startSpeed >= attackSpeed) {
        return 0.0;
    }

    if (acceleration <= 0.0) {
        return VERY_LARGE_TIME;
    }

    return (attackSpeed * attackSpeed - startSpeed * startSpeed) / (2.0 * acceleration);
}

static Coord normalizedOrZero(const Coord& value) {
    double valueLength = length(value);
    if (valueLength <= 0.0) {
        return {};
    }

    return value / valueLength;
}

static double estimateDynamicDistanceMargin(const DroneConfig& config, double currentSpeed) {
    double stepTime = config.physicsTimeStep > 0.0 ? config.physicsTimeStep : config.simTimeStep;
    if (stepTime <= 0.0) {
        return 0.0;
    }

    double referenceSpeed = std::max(currentSpeed, config.attackSpeed);
    return referenceSpeed * stepTime;
}

static bool estimateTimeToCompleteMove(double& totalTime, double& currentSpeed, double& currentDir, const Coord& startPos, const Coord& goalPos, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold) {
    Coord deltaToGoal = goalPos - startPos;
    double distance = length(deltaToGoal);
    if (distance <= 0.0) {
        return true;
    }

    double goalDir = atan2(deltaToGoal.y, deltaToGoal.x);
    double deltaAngle = fabs(calculateAngleDifference(currentDir, goalDir));
    double slowTurnThreshold = turnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    if (deltaAngle > slowTurnThreshold) {
        if (currentSpeed > 0.0) {
            if (acceleration <= 0.0) {
                return false;
            }

            totalTime += currentSpeed / acceleration;
            currentSpeed = 0.0;
        }


        if (angularSpeed <= 0.0) {
            return false;
        }

        totalTime += deltaAngle / angularSpeed;
        currentDir = goalDir;

        totalTime += estimateTravelTime(distance, 0.0, attackSpeed, acceleration);
        currentSpeed = attackSpeed;

        return true;
    }

    if (deltaAngle > turnThreshold) {
        if (angularSpeed <= 0.0) {
            return false;
        }

        totalTime += deltaAngle / angularSpeed;
    }

    currentDir = goalDir;
    totalTime += estimateTravelTime(distance, currentSpeed, attackSpeed, acceleration);
    currentSpeed = attackSpeed;

    return true;
}

static bool estimateTimeToTargetPath(double& totalTime, const Coord& currentPos, double currentSpeed, double currentDir, bool isTurning, bool isMoving, bool isDecelerating, bool isAccelerating, int currentTargetIndex, int targetIndex, double turnTargetDir, double turnRemainingTime, bool needManeuver, const Coord& maneuverPoint, const Coord& dropPoint, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold, double releaseTurnThreshold) {
    totalTime = 0.0;
    double localSpeed = currentSpeed;
    double localDir = currentDir;
    Coord localPos = currentPos;

    if (isTurning && currentTargetIndex == targetIndex) {
        totalTime += turnRemainingTime;
        localSpeed = 0.0;
        localDir = turnTargetDir;
    }

    if (currentTargetIndex != -1 && currentTargetIndex != targetIndex) {
        double timeToStop = calculateTimeToStop(isTurning, currentSpeed, acceleration, turnRemainingTime);
        totalTime += timeToStop;

        if (isMoving || isAccelerating || isDecelerating) {
            if (acceleration > 0.0 && currentSpeed > 0.0) {
                double stopDistance = currentSpeed * currentSpeed / (2.0 * acceleration);
                localPos += {cos(localDir) * stopDistance, sin(localDir) * stopDistance};
            }
        }

        localSpeed = 0.0;
        if (isTurning) {
            localDir = turnTargetDir;
        }
    }

    if (needManeuver) {
        if (!estimateTimeToCompleteMove(totalTime, localSpeed, localDir, localPos, maneuverPoint, attackSpeed, acceleration, angularSpeed, turnThreshold)) {
            return false;
        }
        localPos = maneuverPoint;
    }

    return estimateTimeToCompleteMove(totalTime, localSpeed, localDir, localPos, dropPoint, attackSpeed, acceleration, angularSpeed, releaseTurnThreshold);
}

static double calculateReleaseTurnThreshold(const DroneConfig& config, const BallisticsResult& ballistics) {
    if (ballistics.horizontalDistance <= 0.0 || config.hitRadius <= 0.0) {
        return config.turnThreshold;
    }

    double ratio = config.hitRadius / ballistics.horizontalDistance;
    if (ratio >= 1.0) {
        return config.turnThreshold;
    }

    return std::min(config.turnThreshold, std::asin(ratio));
}

Coord TargetAnalyzer::predictTargetPosition(const Target& target, double flightTime) {
    return target.position + target.velocity * flightTime;
}

void TargetAnalyzer::calculateDropPoint(Coord& dropPoint, Coord& maneuverPoint, bool& needManeuver, const Coord& targetPos, const Coord& dronePos, double horizontalDistance, double accelerationPath) {
    Coord startPos = dronePos;
    double distanceToTarget = distanceBetween(startPos, targetPos);
    needManeuver = (horizontalDistance + accelerationPath > distanceToTarget);
    if (needManeuver) {

        double maneuverDistance = horizontalDistance + accelerationPath;

        if (distanceToTarget == 0.0) {
            maneuverPoint = {targetPos.x - maneuverDistance, targetPos.y};
        } else {
            maneuverPoint = targetPos - (targetPos - startPos) * (maneuverDistance / distanceToTarget);
        }
        startPos = maneuverPoint;
        distanceToTarget = distanceBetween(startPos, targetPos);
    } else {
        maneuverPoint = {};
    }
    if (distanceToTarget > 0.0) {
        double ratio = (distanceToTarget - horizontalDistance) / distanceToTarget;
        dropPoint = startPos + (targetPos - startPos) * ratio;
    } else {
        dropPoint = startPos;
    }
}

bool TargetAnalyzer::evaluateTarget(BestTargetResult& best, const DroneConfig& config, const DroneMotionState& droneMotion, bool isTurning, bool isMoving, bool isDecelerating, bool isAccelerating, const Target& target, int targetIndex, const Coord& dronePosition, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver) {
    Coord dropPoint{};
    Coord maneuverPoint{};
    bool needManeuver = false;
    double totalTime = 0.0;
    Coord predictedTarget = target.position;
    double releaseHeading = droneMotion.currentDir;
    double releaseTurnThreshold = calculateReleaseTurnThreshold(config, ballistics);
    double remainingAccelerationDistance =
        distanceToFullSpeed(droneMotion.currentSpeed, config.attackSpeed, acceleration);

    for (int iteration = 0; iteration < TARGET_PREDICTION_ITERATIONS; ++iteration) {
        predictedTarget = predictTargetPosition(target, totalTime + ballistics.flightTime);
        calculateDropPoint(dropPoint, maneuverPoint, needManeuver, predictedTarget, dronePosition, ballistics.horizontalDistance, remainingAccelerationDistance);
        Coord releaseVector = predictedTarget - dropPoint;
        if (length(releaseVector) > 0.0) {
            releaseHeading = atan2(releaseVector.y, releaseVector.x);
        }

        if (returningFromManuver && targetIndex == droneMotion.currentTargetIndex) {
            needManeuver = false;
        }

        if (!estimateTimeToTargetPath(totalTime, dronePosition, droneMotion.currentSpeed, droneMotion.currentDir, isTurning, isMoving, isDecelerating, isAccelerating, droneMotion.currentTargetIndex, targetIndex, droneMotion.turnTargetDir, droneMotion.turnRemainingTime, needManeuver, maneuverPoint, dropPoint, config.attackSpeed, acceleration, config.angularSpeed, config.turnThreshold, releaseTurnThreshold)) {
            ERROR_LOG("Invalid time estimate to drop point");
            return false;
        }
    }

    double distanceToDropPoint = distanceBetween(dronePosition, dropPoint);
    double requiredAccelerationDistance = distanceToFullSpeed(droneMotion.currentSpeed, config.attackSpeed, acceleration);
    double feasibilityMargin = estimateDynamicDistanceMargin(config, droneMotion.currentSpeed);
    bool alreadyAtAttackSpeed = droneMotion.currentSpeed >= config.attackSpeed - 1e-6;
    if (!alreadyAtAttackSpeed
        && distanceToDropPoint <= requiredAccelerationDistance + feasibilityMargin) {
        double recoveryTotalTime = totalTime;
        Coord recoveryPredictedTarget = predictedTarget;
        Coord recoveryManeuverPoint{};
        Coord recoveryDropPoint{};
        bool recoveryValid = false;
        double fullAccelerationDistance = distanceToFullSpeed(0.0, config.attackSpeed, acceleration);
        Coord awayDirection = normalizedOrZero(dronePosition - predictedTarget);

        if (length(awayDirection) <= 0.0) {
            awayDirection = {-std::cos(droneMotion.currentDir), -std::sin(droneMotion.currentDir)};
        }

        for (int iteration = 0; iteration < TARGET_PREDICTION_ITERATIONS; ++iteration) {
            double deficit = requiredAccelerationDistance + feasibilityMargin - distanceToDropPoint;
            if (deficit < 0.0) {
                deficit = 0.0;
            }

            double recoveryDistance = std::max(deficit, feasibilityMargin);
            recoveryManeuverPoint = dronePosition + awayDirection * recoveryDistance;

            Coord unusedPoint{};
            bool unusedNeedManeuver = false;
            calculateDropPoint(recoveryDropPoint, unusedPoint, unusedNeedManeuver, recoveryPredictedTarget, recoveryManeuverPoint, ballistics.horizontalDistance, fullAccelerationDistance);

            if (!estimateTimeToTargetPath(recoveryTotalTime, dronePosition, droneMotion.currentSpeed, droneMotion.currentDir, isTurning, isMoving, isDecelerating, isAccelerating, droneMotion.currentTargetIndex, targetIndex, droneMotion.turnTargetDir, droneMotion.turnRemainingTime, true, recoveryManeuverPoint, recoveryDropPoint, config.attackSpeed, acceleration, config.angularSpeed, config.turnThreshold, releaseTurnThreshold)) {
                break;
            }

            recoveryPredictedTarget = predictTargetPosition(target, recoveryTotalTime + ballistics.flightTime);
            Coord releaseVector = recoveryPredictedTarget - recoveryDropPoint;
            if (length(releaseVector) > 0.0) {
                releaseHeading = atan2(releaseVector.y, releaseVector.x);
            }
            distanceToDropPoint = distanceBetween(dronePosition, recoveryDropPoint);
            recoveryValid = true;
        }

        if (!recoveryValid) {
            DEBUG("Target " << targetIndex << " rejected: not enough distance to reach attack speed before drop point");
            return false;
        }

        needManeuver = true;
        maneuverPoint = recoveryManeuverPoint;
        dropPoint = recoveryDropPoint;
        predictedTarget = recoveryPredictedTarget;
        totalTime = recoveryTotalTime;
    }

    best.targetIndex = targetIndex;
    best.totalTime = totalTime;
    best.dropPoint = dropPoint;
    best.needManeuver = needManeuver;
    best.maneuverPoint = maneuverPoint;
    best.targetPos = target.position;
    best.predictedTarget = predictedTarget;
    best.releaseHeading = releaseHeading;
    best.releaseTurnThreshold = releaseTurnThreshold;

    return true;
}

bool TargetAnalyzer::selectBestTarget(BestTargetResult& result, const DroneConfig& config, const DroneMotionState& droneMotion, bool isTurning, bool isMoving, bool isDecelerating, bool isAccelerating, const Coord& dronePosition, const ITargetProvider& targets, double simulationTime, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver) {
    result.targetIndex = -1;
    double minTotalTime = VERY_LARGE_TIME;
    double currentTargetTotalTime = VERY_LARGE_TIME;

    for (int targetIndex = 0; targetIndex < targets.getTargetCount(); ++targetIndex) {
        BestTargetResult candidate{};
        if (!evaluateTarget(candidate, config, droneMotion, isTurning, isMoving, isDecelerating, isAccelerating, targets.getTargetAt(targetIndex, simulationTime), targetIndex, dronePosition, ballistics, acceleration, returningFromManuver)) {
            DEBUG("Invalid calculation for target " << targetIndex);
            continue;
        }

        if (targetIndex == droneMotion.currentTargetIndex) {
            currentTargetTotalTime = candidate.totalTime;
        }

        if (candidate.totalTime < minTotalTime) {
            minTotalTime = candidate.totalTime;
            result = candidate;
        }
    }

    if (result.targetIndex == -1) {
        ERROR_LOG("Failed to find the best target");
        return false;
    }

    if (result.targetIndex != droneMotion.currentTargetIndex && droneMotion.currentTargetIndex != -1) {
        if (currentTargetTotalTime < VERY_LARGE_TIME && currentTargetTotalTime <= minTotalTime) {
            if (!evaluateTarget(result, config, droneMotion, isTurning, isMoving, isDecelerating, isAccelerating, targets.getTargetAt(droneMotion.currentTargetIndex, simulationTime), droneMotion.currentTargetIndex, dronePosition, ballistics, acceleration, returningFromManuver)) {
                DEBUG("Failed to keep focus on the current target");
                return false;
            }
        }
    }

    return true;
}
