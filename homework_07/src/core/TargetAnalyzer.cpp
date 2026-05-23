#include "core/TargetAnalyzer.hpp"
#include "utils/logger.hpp"
#include "domain/types.hpp"

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
