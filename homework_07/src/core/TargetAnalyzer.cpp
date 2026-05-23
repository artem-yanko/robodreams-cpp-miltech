#include "core/TargetAnalyzer.hpp"
#include "utils/logger.hpp"
#include "domain/types.hpp"
#include <cmath>

static const double VERY_LARGE_TIME = 1e18;
static const double TARGET_SWITCH_PREVENTION = 1.0;

static bool interpolateTargetPosition(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep, Coord& targetPos) {
  if (arrayTimeStep == 0.0) {
    ERROR_LOG("Крок часу масиву цілей не може дорівнювати нулю!");
    return false;
  }
  if (targetIndex < 0 || targetIndex >= targets.targetCount) {
    ERROR_LOG("Невірний індекс цілі: " << targetIndex);
    return false;
  }
  if (simulationTime < 0) {
    ERROR_LOG("Час не може бути від'ємним!");
    return false;
  }


  double targetTime = simulationTime / arrayTimeStep;
  int baseIdx = (int)floor(targetTime);
  int idx = baseIdx % targets.timeSteps;
  int next = (idx + 1) % targets.timeSteps;
  double frac = targetTime - baseIdx;
  targetPos = targets.positions[targetIndex][idx] + (targets.positions[targetIndex][next] - targets.positions[targetIndex][idx]) * frac;
  return true;
}

static bool calculateTargetVelocity(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep, Coord& velocity) {
  if (arrayTimeStep == 0.0) {
    ERROR_LOG("Крок часу масиву цілей не може дорівнювати нулю!");
    return false;
  }

  double dt = arrayTimeStep;

  Coord firstPos{};
  if (!interpolateTargetPosition(targetIndex, targets, simulationTime, arrayTimeStep, firstPos)) {
    return false;
  }

  Coord secondPos{};
  if (!interpolateTargetPosition(targetIndex, targets, simulationTime + dt, arrayTimeStep, secondPos)) {
    return false;
  }

  velocity = (secondPos - firstPos) / dt;

  return true;
}

static double length(const Coord& c) {
  return hypot(c.x, c.y);
}

static double distanceBetween(const Coord& from, const Coord& to) {
  return length(to - from);
}

static Coord normalize(const Coord& c) {
  double len = length(c);
  if (len == 0.0) {
      return {};
  }
  return c / len;
}

static double calculateTimeToStop(DronePhase phase, double currentSpeed, double acceleration, double turnRemainingTime) {
  if (phase == STOPPED) {
    return 0.0;
  }

  if (phase == TURNING) {
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

static double calculateAngleDifference(double currentDir, double targetDir) {
  double delta = targetDir - currentDir;

  if (delta > M_PI) {
    delta -= 2.0 * M_PI;
  }
  if (delta < -M_PI) {
    delta += 2.0 * M_PI;
  }

  return delta;
}

static bool estimateTimeToCompleteMove(double& totalTime, double& currentSpeed, double& currentDir, const Coord& startPos, const Coord& goalPos, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold) {
  Coord deltaToGoal = goalPos - startPos;
  double distance = length(deltaToGoal);
  if (distance <= 0.0) {
    return true;
  }

  double goalDir = atan2(deltaToGoal.y, deltaToGoal.x);
  double deltaAngle = fabs(calculateAngleDifference(currentDir, goalDir));
  if (deltaAngle > turnThreshold) {
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

  currentDir = goalDir;
  totalTime += estimateTravelTime(distance, currentSpeed, attackSpeed, acceleration);
  currentSpeed = attackSpeed;

  return true;
}

static bool estimateTimeToTargetPath(double& totalTime, const Coord& currentPos, double currentSpeed, double currentDir, DronePhase currentPhase, int currentTargetIndex, int targetIndex, double turnTargetDir, double turnRemainingTime, bool needManeuver, const Coord& maneuverPoint, const Coord& dropPoint, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold) {
  totalTime = 0.0;
  double localSpeed = currentSpeed;
  double localDir = currentDir;
  Coord localPos = currentPos;

  if (currentPhase == TURNING && currentTargetIndex == targetIndex) {
    totalTime += turnRemainingTime;
    localSpeed = 0.0;
    localDir = turnTargetDir;
  }

  if (currentTargetIndex != -1 && currentTargetIndex != targetIndex) {
    double timeToStop = calculateTimeToStop(currentPhase, currentSpeed, acceleration, turnRemainingTime);
    totalTime += timeToStop;

    if (currentPhase == MOVING || currentPhase == ACCELERATING || currentPhase == DECELERATING) {
      if (acceleration > 0.0 && currentSpeed > 0.0) {
        double stopDistance = currentSpeed * currentSpeed / (2.0 * acceleration);
        localPos += {cos(localDir) * stopDistance, sin(localDir) * stopDistance};
      }
    }

    localSpeed = 0.0;
    if (currentPhase == TURNING) {
      localDir = turnTargetDir;
    }
  }

  if (needManeuver) {
    if (!estimateTimeToCompleteMove(totalTime, localSpeed, localDir, localPos, maneuverPoint, attackSpeed, acceleration, angularSpeed, turnThreshold)) {
      return false;
    }
    // TODO: Перенести оновлення координат в estimateTimeToCompleteMove.
    localPos = maneuverPoint;
  }

  if (!estimateTimeToCompleteMove(totalTime, localSpeed, localDir, localPos, dropPoint, attackSpeed, acceleration, angularSpeed, turnThreshold)) { 
    return false;
    // Якщо буде продовження симуляції, оновити позицію, або перенести в estimateTimeToCompleteMove
  }

  return true;
}

Target TargetAnalyzer::analyzeTarget(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep) {
  Target result{};

  if (!interpolateTargetPosition(targetIndex, targets, simulationTime, arrayTimeStep, result.position)) {
    ERROR_LOG("Помилка при інтерполяції позиції цілі.");
    return result;
  }

  if (!calculateTargetVelocity(targetIndex, targets, simulationTime, arrayTimeStep, result.velocity)) {
    ERROR_LOG("Помилка при розрахунку швидкості цілі.");
    return result;
  }

  return result;
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
  if (distanceToTarget > 0) {
    double ratio = (distanceToTarget - horizontalDistance) / distanceToTarget;
    dropPoint = startPos + (targetPos - startPos) * ratio;
  } else {
    dropPoint = startPos;
  }
}

bool TargetAnalyzer::evaluateTarget(BestTargetResult& best, const DroneConfig& config, const DroneMotionState& droneMotion, const TargetData& targets, int targetIndex, const Coord& dronePosition, double simulationTime, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver) {
  Target target = analyzeTarget(targetIndex, targets, simulationTime, config.arrayTimeStep);
  Coord predictedTarget = predictTargetPosition(target, ballistics.flightTime);
  Coord dropPoint{};
  Coord maneuverPoint{};
  bool needManeuver = false;
  double totalTime = 0.0;

  calculateDropPoint(dropPoint, maneuverPoint, needManeuver, predictedTarget, dronePosition, ballistics.horizontalDistance, config.accelPath);

  if (returningFromManuver && targetIndex == droneMotion.currentTargetIndex) {
    needManeuver = false;
  }

  if (!estimateTimeToTargetPath(totalTime, dronePosition, droneMotion.currentSpeed, droneMotion.currentDir, droneMotion.phase, droneMotion.currentTargetIndex, targetIndex, droneMotion.turnTargetDir, droneMotion.turnRemainingTime, needManeuver, maneuverPoint, dropPoint, config.attackSpeed, acceleration, config.angularSpeed, config.turnThreshold)) {
    ERROR_LOG("Невірна оцінка часу до точки скиду");
    return false;
  }

  best.targetIndex = targetIndex;
  best.totalTime = totalTime;
  best.dropPoint = dropPoint;
  best.needManeuver = needManeuver;
  best.maneuverPoint = maneuverPoint;
  best.targetPos = target.position;
  best.predictedTarget = predictedTarget;

  return true;
}

bool TargetAnalyzer::selectBestTarget(BestTargetResult& result, const DroneConfig& config, const DroneMotionState& droneMotion, const Coord& dronePosition, const TargetData& targets, double simulationTime, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver) {
  result.targetIndex = -1;
  double minTotalTime = VERY_LARGE_TIME;
  double currentTargetTotalTime = VERY_LARGE_TIME;

  for (int targetIndex = 0; targetIndex < targets.targetCount; ++targetIndex) {
    BestTargetResult candidate{};
    if (!evaluateTarget(candidate, config, droneMotion, targets, targetIndex, dronePosition, simulationTime, ballistics, acceleration, returningFromManuver)) {
      DEBUG("Невірний розрахунок для цілі " << targetIndex);
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
    ERROR_LOG("Не знайдено найкращої цілі");
    return false;
  }

  if (result.targetIndex != droneMotion.currentTargetIndex && droneMotion.currentTargetIndex != -1) {
    if (currentTargetTotalTime < VERY_LARGE_TIME && minTotalTime > currentTargetTotalTime - TARGET_SWITCH_PREVENTION) {
      if (!evaluateTarget(result, config, droneMotion, targets, droneMotion.currentTargetIndex, dronePosition, simulationTime, ballistics, acceleration, returningFromManuver)) {
        DEBUG("Не вдалося сфокусуватися на цілі");
        return false;
      }
    }
  }

  if (!interpolateTargetPosition(result.targetIndex, targets, simulationTime, config.arrayTimeStep, result.targetPos)) {
    ERROR_LOG("Невірний розрахунок позиції найкращої цілі");
    return false;
  }

  return true;
}
