#include "core/MissionProcessor.hpp"
#include "core/TargetAnalyzer.hpp"
#include "utils/logger.hpp"
#include "utils/math_utils.hpp"
#include <cstring>
#include <cmath>

static const double SLOW_TURN_THRESHOLD_FACTOR = 1.0;

static double calculateDroneAcceleration(double attackSpeed, double accelerationPath) {
  if (accelerationPath <= 0.0) {
    return 0.0;
  }
  return attackSpeed * attackSpeed / (2.0 * accelerationPath);
}

static void moveDroneToStop(Coord& position, double direction, double distance) {
  position.x += cos(direction) * distance;
  position.y += sin(direction) * distance;
}

static bool moveDroneToPoint(Coord& position, const Coord& dest, double maxDistance){
  Coord delta = dest - position;
  double distance = length(delta);
  if (distance == 0.0) {
    return true;
  }

  if (maxDistance >= distance) {
      position = dest;
  return true;
  }

  position += normalize(delta) * maxDistance;
  return false;
}

static bool updateDroneMotion(Coord& dronePosition, DroneMotionState& droneMotion, const Coord& goal, double simTimeStep, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold) {

  Coord deltaToGoal = goal - dronePosition;
  double desiredDir = droneMotion.currentDir;

  if (deltaToGoal.x != 0.0 || deltaToGoal.y != 0.0) {
    desiredDir = atan2(deltaToGoal.y, deltaToGoal.x);
  }

  if (droneMotion.phase != TURNING) {
    double deltaAngle = fabs(calculateAngleDifference(droneMotion.currentDir, desiredDir));
    double slowTurnThreshold = turnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    bool shouldStopForTurn = deltaAngle > turnThreshold;
    if (droneMotion.phase == MOVING) {
      shouldStopForTurn = deltaAngle > slowTurnThreshold;
    }

    if (shouldStopForTurn) {
      droneMotion.turnTargetDir = desiredDir;
      if (droneMotion.currentSpeed > 0.0) {
        droneMotion.phase = DECELERATING;
      } else {
        droneMotion.phase = TURNING;
        droneMotion.turnRemainingTime = deltaAngle / angularSpeed;
      }
    } else {
      if (deltaAngle <= turnThreshold) {
        droneMotion.currentDir = desiredDir;
      }
      if (droneMotion.phase == STOPPED) {
        droneMotion.phase = ACCELERATING;
      }
    }
  }

  bool reachedGoal = false;

  if (droneMotion.phase == DECELERATING) {
    double oldSpeed = droneMotion.currentSpeed;
    droneMotion.currentSpeed -= acceleration * simTimeStep;
    if (droneMotion.currentSpeed < 0.0) {
      droneMotion.currentSpeed = 0.0;
    }

    double moveDistance = (oldSpeed + droneMotion.currentSpeed) * simTimeStep / 2.0;
    moveDroneToStop(dronePosition, droneMotion.currentDir, moveDistance);
    if (droneMotion.currentSpeed == 0.0) {
      double turnAngle = fabs(calculateAngleDifference(droneMotion.currentDir, desiredDir));
      droneMotion.turnTargetDir = desiredDir;
      if (turnAngle > 0.0) {
        droneMotion.phase = TURNING;
        droneMotion.turnRemainingTime = turnAngle / angularSpeed;
      } else {
        droneMotion.phase = ACCELERATING;
        droneMotion.turnRemainingTime = 0.0;
      }
    }
  } else if (droneMotion.phase == TURNING) {
    double angleLeft = calculateAngleDifference(droneMotion.currentDir, droneMotion.turnTargetDir);
    double turnStep = angularSpeed * simTimeStep;

    if (fabs(angleLeft) <= turnStep) {
      droneMotion.currentDir = droneMotion.turnTargetDir;
      droneMotion.turnRemainingTime = 0.0;
      droneMotion.phase = ACCELERATING;
    } else {
      if (angleLeft > 0.0) {
        droneMotion.currentDir += turnStep;
        if (droneMotion.currentDir > M_PI) {
          droneMotion.currentDir -= 2.0 * M_PI;
        }
      } else {
        droneMotion.currentDir -= turnStep;
        if (droneMotion.currentDir < -M_PI) {
          droneMotion.currentDir += 2.0 * M_PI;
        }
      }
      droneMotion.turnRemainingTime = (fabs(angleLeft) - turnStep) / angularSpeed;
    }
  } else if (droneMotion.phase == ACCELERATING) {

    double oldSpeed = droneMotion.currentSpeed;
    droneMotion.currentSpeed += acceleration * simTimeStep;
    if (droneMotion.currentSpeed > attackSpeed) {
      droneMotion.currentSpeed = attackSpeed;
    }

    double moveDistance = (oldSpeed + droneMotion.currentSpeed) * simTimeStep / 2.0;
    reachedGoal = moveDroneToPoint(dronePosition, goal, moveDistance);

    if (droneMotion.currentSpeed >= attackSpeed) {
      droneMotion.phase = MOVING;
    }
  } else if (droneMotion.phase == MOVING) {
    double angleLeft = calculateAngleDifference(droneMotion.currentDir, desiredDir);
    double turnStep = angularSpeed * simTimeStep;
    double slowTurnThreshold = turnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    if (fabs(angleLeft) <= slowTurnThreshold) {
      if (fabs(angleLeft) <= turnStep) {
        droneMotion.currentDir = desiredDir;
      } else if (angleLeft > 0.0) {
        droneMotion.currentDir += turnStep;
        if (droneMotion.currentDir > M_PI) {
          droneMotion.currentDir -= 2.0 * M_PI;
        }
      } else {
        droneMotion.currentDir -= turnStep;
        if (droneMotion.currentDir < -M_PI) {
          droneMotion.currentDir += 2.0 * M_PI;
        }
      }
    }

    droneMotion.currentSpeed = attackSpeed;
    reachedGoal = moveDroneToPoint(dronePosition, goal, droneMotion.currentSpeed * simTimeStep);
  } else if (droneMotion.phase == STOPPED) {
    droneMotion.currentSpeed = 0.0;

    double deltaAngle = fabs(calculateAngleDifference(droneMotion.currentDir, desiredDir));
    if (deltaAngle > turnThreshold) {
      droneMotion.phase = TURNING;
      droneMotion.turnTargetDir = desiredDir;
      droneMotion.turnRemainingTime = deltaAngle / angularSpeed;
    } else {
      droneMotion.currentDir = desiredDir;
      droneMotion.phase = ACCELERATING;
    }
  }

  return reachedGoal;
}

MissionProcessor::~MissionProcessor() {
    if (ammoList) {
        delete[] ammoList;
        ammoList = nullptr;
    }
}

MissionProcessor::MissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader) : targets(targets), solver(solver), configLoader(configLoader) {
        ammoList = nullptr;
        ammoCount = 0;
        currentIndex = 0;
        simulationTime = 0.0;
    }

bool MissionProcessor::init() {
    currentIndex = 0;
    simulationTime = 0.0;
    if (!configLoader->loadConfig(config)) {
        ERROR_LOG("Failed to load drone configuration");
        return false;
    }
    LOG("Configuration loaded: attackSpeed=" << config.attackSpeed
        << ", accelPath=" << config.accelPath
        << ", ammoName=" << config.ammoName
        << ", startPos=(" << config.startPos.x << "," << config.startPos.y << ")");

    if (!configLoader->loadAmmo(ammoList, ammoCount)) {
        ERROR_LOG("Failed to load ammo parameters");
        return false;
    }
    LOG("Ammo parameters loaded: " << ammoCount);

    if (!targets->loadTargets()) {
        ERROR_LOG("Failed to load targets");
        return false;
    }

    return true;
}

bool MissionProcessor::hasNext() {
    return currentIndex < targets->getTargetCount();
}

static const AmmoParams* ammoSelect(const AmmoParams* ammoList, int ammoCount, const char* ammoName) {
    for (int i = 0; i < ammoCount; ++i) {
        if (strcmp(ammoName, ammoList[i].name) == 0) {
            return &ammoList[i];
        }
    }
    return nullptr;
}

BallisticsResult MissionProcessor::step() {
    TargetAnalyzer analyzer;
    
    const AmmoParams* selectedAmmo = ammoSelect(ammoList, ammoCount, config.ammoName);
    if (selectedAmmo == nullptr) {
        ERROR_LOG("Failed to select ammo parameters");
        return {};
    }
    BallisticsResult result = solver->solve(config, *selectedAmmo);
    BestTargetResult best{};
    DroneMotionState droneMotion{};
    double acceleration = 0.0;
    bool returningFromManuver = false;

    if (analyzer.selectBestTarget(best, config, droneMotion, config.startPos, targets->getTargetsData(), simulationTime, result, acceleration, returningFromManuver)) {
        DEBUG("Selected target " << best.targetIndex
            << ": totalTime=" << best.totalTime
            << ", dropPoint=(" << best.dropPoint.x << "," << best.dropPoint.y << ")"
            << ", predictedTarget=(" << best.predictedTarget.x << "," << best.predictedTarget.y << ")"
            << ", needManeuver=" << best.needManeuver);
    }
        
        currentIndex++;
        simulationTime += config.simTimeStep;
        return result;
}

void MissionProcessor::reset() {
    currentIndex = 0;
    simulationTime = 0.0;
}

void MissionProcessor::changeSolver(IBallisticSolver* newSolver) {
    solver = newSolver;
}
