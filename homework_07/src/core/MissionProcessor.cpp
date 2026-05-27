#include "core/MissionProcessor.hpp"
#include "core/TargetAnalyzer.hpp"
#include "utils/logger.hpp"
#include "utils/math_utils.hpp"
#include "utils/json.hpp"
#include <cmath>
#include <fstream>

static const double SLOW_TURN_THRESHOLD_FACTOR = 1.0;

static bool writeSimulationJson(const std::vector<SimStep>& steps) {
  std::ofstream outputFile("simulation.json");
  if (!outputFile) {
    ERROR_LOG("Failed to open \"simulation.json\" for writing");
    return false;
  }

  nlohmann::ordered_json outputJson;
  outputJson["totalSteps"] = steps.empty() ? 0 : static_cast<int>(steps.size()) - 1;
  outputJson["steps"] = nlohmann::ordered_json::array();

  for (const SimStep& step : steps) {
    nlohmann::ordered_json stepJson;
    stepJson["position"] = {
      {"x", step.pos.x},
      {"y", step.pos.y}
    };
    stepJson["direction"] = step.direction;
    stepJson["state"] = step.state;
    stepJson["targetIndex"] = step.targetIdx;
    stepJson["dropPoint"] = {
      {"x", step.dropPoint.x},
      {"y", step.dropPoint.y}
    };
    stepJson["aimPoint"] = {
      {"x", step.aimPoint.x},
      {"y", step.aimPoint.y}
    };
    stepJson["predictedTarget"] = {
      {"x", step.predictedTarget.x},
      {"y", step.predictedTarget.y}
    };
    outputJson["steps"].push_back(stepJson);
  }

  outputFile << outputJson.dump(2);
  return true;
}

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

static bool isInsideRadius(const Coord& point, const Coord& center, double radius) {
  return distanceBetween(point, center) <= radius;
}

static const AmmoParams* ammoSelect(const std::vector<AmmoParams>& ammoList, const std::string& ammoName) {
    for (const auto& ammo : ammoList) {
        if (ammoName == ammo.name) {
            return &ammo;
        }
    }
    return nullptr;
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

MissionProcessor::~MissionProcessor() = default;

MissionProcessor::MissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader) : targets(targets), solver(solver), configLoader(configLoader) {
        ballistics = {};
        simulationTime = 0.0;
        acceleration = 0.0;
        missionComplete = false;
        targetLocked = false;
        lockedTargetIndex = -1;
        returningFromManuver = false;
        maneuverTargetIndex = -1;
    }

bool MissionProcessor::init() {
    simulationTime = 0.0;
    if (!configLoader->loadConfig(config)) {
        ERROR_LOG("Failed to load drone configuration");
        return false;
    }
    LOG("Configuration loaded: attackSpeed=" << config.attackSpeed
        << ", accelPath=" << config.accelPath
        << ", ammoName=" << config.ammoName
        << ", startPos=(" << config.startPos.x << "," << config.startPos.y << ")");

    ammoList.clear();

    if (!configLoader->loadAmmo(ammoList)) {
        ERROR_LOG("Failed to load ammo parameters");
        return false;
    }
    LOG("Ammo parameters loaded: " << ammoList.size());

    const AmmoParams* selectedAmmo = ammoSelect(ammoList, config.ammoName);
    if (selectedAmmo == nullptr) {
        ERROR_LOG("Failed to select ammo parameters");
        return false;
    }
    ballistics = solver->solve(config, *selectedAmmo);

    if (!targets->loadTargets()) {
        ERROR_LOG("Failed to load targets");
        return false;
    }

    dronePosition = config.startPos;
    acceleration = calculateDroneAcceleration(config.attackSpeed, config.accelPath);
    droneMotion.currentSpeed = 0.0;
    droneMotion.currentDir = config.initialDir;
    droneMotion.turnTargetDir = config.initialDir;
    droneMotion.turnRemainingTime = 0.0;
    droneMotion.phase = STOPPED;
    droneMotion.currentTargetIndex = -1;
    missionComplete = false;
    targetLocked = false;
    lockedTargetIndex = -1;
    returningFromManuver = false;
    maneuverTargetIndex = -1;
    steps.clear();

    SimStep initialStep{};
    initialStep.pos = dronePosition;
    initialStep.direction = droneMotion.currentDir;
    initialStep.state = droneMotion.phase;
    initialStep.targetIdx = -1;
    steps.push_back(initialStep);

    return true;
}

bool MissionProcessor::hasNext() {
    return !missionComplete;
}

BallisticsResult MissionProcessor::step() {
    TargetAnalyzer analyzer;
    BestTargetResult best{};

    bool targetSelected = false;
    if (targetLocked) {
        targetSelected = analyzer.evaluateTarget(best, config, droneMotion, targets->getTargetsData(), lockedTargetIndex, dronePosition, simulationTime, ballistics, acceleration, returningFromManuver);
        if (targetSelected) {
            DEBUG("Locked target " << best.targetIndex
                << ": totalTime=" << best.totalTime
                << ", dropPoint=(" << best.dropPoint.x << "," << best.dropPoint.y << ")"
                << ", predictedTarget=(" << best.predictedTarget.x << "," << best.predictedTarget.y << ")"
                << ", needManeuver=" << best.needManeuver);
        }
    } else {
        targetSelected = analyzer.selectBestTarget(best, config, droneMotion, dronePosition, targets->getTargetsData(), simulationTime, ballistics, acceleration, returningFromManuver);
        if (targetSelected) {
            DEBUG("Selected target " << best.targetIndex
                << ": totalTime=" << best.totalTime
                << ", dropPoint=(" << best.dropPoint.x << "," << best.dropPoint.y << ")"
                << ", predictedTarget=(" << best.predictedTarget.x << "," << best.predictedTarget.y << ")"
                << ", needManeuver=" << best.needManeuver);
        }
    }

    if (!targetSelected) {
        ERROR_LOG("Failed to evaluate targets");
        missionComplete = true;
        return ballistics;
    }

    droneMotion.currentTargetIndex = best.targetIndex;

    if (best.targetIndex != maneuverTargetIndex) {
        returningFromManuver = false;
    }
    if (droneMotion.phase == DECELERATING) {
        returningFromManuver = false;
    }
    if (droneMotion.phase == MOVING && !best.needManeuver) {
        returningFromManuver = true;
    }
    if (returningFromManuver) {
        best.needManeuver = false;
    }

    bool atManeuverPoint = best.needManeuver && isInsideRadius(dronePosition, best.maneuverPoint, config.hitRadius);
    if (atManeuverPoint) {
        returningFromManuver = true;
        best.needManeuver = false;
        targetLocked = true;
        lockedTargetIndex = best.targetIndex;
        DEBUG("Reached maneuver point for target " << best.targetIndex);
    }

    Coord goal{};
    bool headingToManeuver = best.needManeuver && !atManeuverPoint;
    if (headingToManeuver) {
        goal = best.maneuverPoint;
        maneuverTargetIndex = best.targetIndex;
        targetLocked = true;
        lockedTargetIndex = best.targetIndex;
    } else {
        goal = best.dropPoint;
    }

    double distanceToDropPoint = distanceBetween(dronePosition, best.dropPoint);
    if (!headingToManeuver && droneMotion.phase == MOVING && !targetLocked && distanceToDropPoint <= ballistics.horizontalDistance) {
        targetLocked = true;
        lockedTargetIndex = best.targetIndex;
        DEBUG("Target locked: " << lockedTargetIndex
            << ", distanceToDropPoint=" << distanceToDropPoint
            << ", horizontalDistance=" << ballistics.horizontalDistance);
    }

    updateDroneMotion(dronePosition, droneMotion, goal, config.simTimeStep, config.attackSpeed, acceleration, config.angularSpeed, config.turnThreshold);
    DEBUG("Drone position: (" << dronePosition.x << "," << dronePosition.y << ")"
        << ", dir=" << droneMotion.currentDir
        << ", speed=" << droneMotion.currentSpeed
        << ", phase=" << droneMotion.phase);

    Coord currentDir{cos(droneMotion.currentDir), sin(droneMotion.currentDir)};
    SimStep currentStep{};
    currentStep.pos = dronePosition;
    currentStep.direction = droneMotion.currentDir;
    currentStep.state = droneMotion.phase;
    currentStep.targetIdx = best.targetIndex;
    currentStep.dropPoint = best.dropPoint;
    currentStep.predictedTarget = best.predictedTarget;
    currentStep.aimPoint = dronePosition + currentDir * ballistics.horizontalDistance;
    steps.push_back(currentStep);

    bool dropNow = !headingToManeuver && isInsideRadius(dronePosition, best.dropPoint, config.hitRadius)
                   && droneMotion.phase == MOVING;
    if (dropNow) {
        LOG("Drop condition met"
            << ", target #" << best.targetIndex
            << ", drone=(" << dronePosition.x << "," << dronePosition.y << ")"
            << ", dropPoint=(" << best.dropPoint.x << "," << best.dropPoint.y << ")");
        if (!writeSimulationJson(steps)) {
            ERROR_LOG("Failed to write simulation.json");
        } else {
            LOG("Simulation written to simulation.json");
        }
        targetLocked = false;
        lockedTargetIndex = -1;
        returningFromManuver = false;
        maneuverTargetIndex = -1;
        missionComplete = true;
        simulationTime += config.simTimeStep;
        return ballistics;
    }
    
    simulationTime += config.simTimeStep;
    return ballistics;
}

void MissionProcessor::reset() {
    simulationTime = 0.0;
    dronePosition = config.startPos;
    droneMotion.currentSpeed = 0.0;
    droneMotion.currentDir = config.initialDir;
    droneMotion.turnTargetDir = config.initialDir;
    droneMotion.turnRemainingTime = 0.0;
    droneMotion.phase = STOPPED;
    droneMotion.currentTargetIndex = -1;
    missionComplete = false;
    targetLocked = false;
    lockedTargetIndex = -1;
    returningFromManuver = false;
    maneuverTargetIndex = -1;
    steps.clear();

    SimStep initialStep{};
    initialStep.pos = dronePosition;
    initialStep.direction = droneMotion.currentDir;
    initialStep.state = droneMotion.phase;
    initialStep.targetIdx = -1;
    steps.push_back(initialStep);
}

void MissionProcessor::changeSolver(IBallisticSolver* newSolver) {
    solver = newSolver;
    if (solver == nullptr || ammoList.empty()) {
        ballistics = {};
        return;
    }

    const AmmoParams* selectedAmmo = ammoSelect(ammoList, config.ammoName);
    if (selectedAmmo == nullptr) {
        ERROR_LOG("Failed to select ammo parameters");
        ballistics = {};
        return;
    }

    ballistics = solver->solve(config, *selectedAmmo);
}
