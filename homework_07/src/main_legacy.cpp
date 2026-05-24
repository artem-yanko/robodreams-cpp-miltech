#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include "json.hpp"
using json = nlohmann::json;

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0
#define ENABLE_ERROR 1

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

#if ENABLE_ERROR
  #define ERROR_LOG(msg) std::cerr << "[ERROR] " << msg << std::endl
#else
  #define ERROR_LOG(msg)
#endif

const double gravity = 9.81;
const int MAX_STEPS = 10000;
const double VERY_LARGE_TIME = 1e18;
const double TARGET_SWITCH_PREVENTION = 1.0;
const double SLOW_TURN_THRESHOLD_FACTOR = 3.0; //TODO: Розробити динамічний коефіцієнт для повільного повороту

enum DronePhase {
  STOPPED = 0,
  ACCELERATING = 1,
  DECELERATING = 2,
  TURNING = 3,
  MOVING = 4
};

struct AmmoParams {
  char name[32]{};
  double mass{};
  double drag{};
  double lift{};
};

struct Coord {
    double x{};
    double y{};

    Coord operator+(const Coord& other) const {
      return {x + other.x, y + other.y};
    }

    Coord operator-(const Coord& other) const {
      return {x - other.x, y - other.y};
    }

    Coord operator*(double scalar) const {
      return {x * scalar, y * scalar};
    }

    Coord operator/(double scalar) const {
      return {x / scalar, y / scalar};
    }

    bool operator==(const Coord& other) const {
      return x == other.x && y == other.y;
    }

    Coord& operator+=(const Coord& other) {
      x += other.x;
      y += other.y;
      return *this;
    }
  };

struct DroneConfig {
  Coord startPos{};
  double altitude{};
  double initialDir{};
  double attackSpeed{};
  double accelPath{};
  char ammoName[32]{};
  double arrayTimeStep{};
  double simTimeStep{};
  double hitRadius{};
  double angularSpeed{};
  double turnThreshold{};
};

struct SimStep {
  Coord pos{};
  double direction{};
  DronePhase state{STOPPED};
  int targetIdx{-1};
  Coord dropPoint{};
  Coord aimPoint{};
  Coord predictedTarget{};
};

struct BallisticsResult {
  double flightTime{};
  double horizontalDistance{};
};

struct BestTargetResult {
  int targetIndex{-1};
  double totalTime{};
  Coord dropPoint{};
  bool needManeuver{};
  Coord maneuverPoint{};
  Coord targetPos{};
  Coord predictedTarget{};
};

struct DroneMotionState {
  double currentSpeed{};
  double currentDir{};
  double turnRemainingTime{};
  double turnTargetDir{};
  DronePhase phase{STOPPED};
  int currentTargetIndex{-1};
};

struct TargetData {
  int targetCount{};
  int timeSteps{};
  Coord** positions{};
};
// довжина вектора в 2D
double length(const Coord& c) {
  return hypot(c.x, c.y);
}
// обрізка вектора до одиничного
Coord normalize(const Coord& c) {
  double len = length(c);
  if (len == 0.0) {
      return {};
  }
  return c / len;
}

const AmmoParams* ammoSelect(const AmmoParams* ammoList, int ammoCount, const char* ammoName) {
  for (int i = 0; i < ammoCount; ++i) {
    if (strcmp(ammoName, ammoList[i].name) == 0) {
      return &ammoList[i];
    }
  }
  return nullptr;
}

double distanceBetween(const Coord& from, const Coord& to) {
  return length(to - from);
}

bool calculateTimeOfFlight(const AmmoParams& ammo, double attackSpeed, double zd, double& flightTime) {
  const double mass = ammo.mass;
  const double drag = ammo.drag;
  const double lift = ammo.lift;

  double a = drag * gravity * mass - 2 * drag * drag * lift * attackSpeed;
  double b = -3 * gravity * mass * mass + 3 * drag * lift * mass * attackSpeed;
  double c = 6 * mass * mass * zd;

  if (a == 0.0) {
    return false;
  }

  double p = -((b * b) / (3 * a * a));
  double q = (2 * b * b * b) / (27 * a * a * a) + c / a;

  if (p == 0.0) {
    return false;
  }

  double acosArg1 = ((3 * q) / (2 * p));

  if ((-3 / p) < 0) {
    return false;
  }

  double acosArg2 = sqrt(-3 / p);

  double acosArg = acosArg1 * acosArg2;
  if (acosArg > 1 || acosArg < -1) {
    ERROR_LOG("Невірне значення для арккосинуса: " << acosArg);
    return false;
  }

  double phi = acos(acosArg);

  flightTime = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);
  if (flightTime <= 0) {
    ERROR_LOG("Невірний розрахунок часу польоту: " << flightTime);
    return false;
  }

  return true;
}

double calculateHorizontalDistance(const AmmoParams& ammo, double attackSpeed, double time) {
  const double mass = ammo.mass;
  const double drag = ammo.drag;
  const double lift = ammo.lift;

  double time2 = time * time;
  double time3 = time2 * time;
  double time4 = time3 * time;
  double time5 = time4 * time;

  double drag2 = drag * drag;
  double drag3 = drag2 * drag;
  double drag4 = drag3 * drag;

  double lift2 = lift * lift;
  double lift3 = lift2 * lift;
  double lift4 = lift3 * lift;

  double mass2 = mass * mass;
  double mass3 = mass2 * mass;
  double mass4 = mass3 * mass;

  // V₀t − t²d·V₀/(2m)
  double part1 = attackSpeed * time - ((time2 * (drag * attackSpeed))) / (2.0 * mass);
  // t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²)
  double part2 = (time3 * (6.0 * drag * gravity * lift * mass - 6.0 * drag2 * (lift2 - 1) * attackSpeed)) / (36.0 * mass2);
  // t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀) / (36(1+l²)²m³)
  double part3 = (time4 * ((-6.0 * drag2 * gravity * lift * (1 + lift2 + lift4)) * mass + (3.0 * drag3 * lift2 * (1 + lift2) * attackSpeed) + (6.0 * drag3 * lift4 * (1 + lift2) * attackSpeed))) / (36.0 * pow(1.0 + lift2, 2) * mass3);
  // t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
  double part4 = (time5 * (3.0 * drag3 * gravity * lift3 * mass - 3.0 * drag4 * lift2 * (1 + lift2) * attackSpeed)) / (36.0 * (1.0 + lift2) * mass4);

  return part1 + part2 + part3 + part4;
}

bool calculateBallistics(BallisticsResult& result, const AmmoParams& ammo, double attackSpeed, double altitude) {
  result.flightTime = 0.0;
  result.horizontalDistance = 0.0;

  if (!calculateTimeOfFlight(ammo, attackSpeed, altitude, result.flightTime)) {
    return false;
  }

  result.horizontalDistance = calculateHorizontalDistance(ammo, attackSpeed, result.flightTime);
  return true;
}

bool readConfigJson(DroneConfig& config) {
  std::ifstream configFile("config.json");
  if (!configFile) {
    ERROR_LOG("Не вдалося відкрити файл \"config.json\"");
    return false;
  }

  json configJson;
  configFile >> configJson;

  config.startPos.x = configJson["drone"]["position"]["x"];
  config.startPos.y = configJson["drone"]["position"]["y"];
  config.altitude = configJson["drone"]["altitude"];
  config.initialDir = configJson["drone"]["initialDirection"];
  config.attackSpeed = configJson["drone"]["attackSpeed"];
  config.accelPath = configJson["drone"]["accelerationPath"];
  config.angularSpeed = configJson["drone"]["angularSpeed"];
  config.turnThreshold = configJson["drone"]["turnThreshold"];

  std::strncpy(config.ammoName, configJson["ammo"].get<std::string>().c_str(), sizeof(config.ammoName) - 1);
  config.ammoName[sizeof(config.ammoName) - 1] = '\0';

  config.arrayTimeStep = configJson["targetArrayTimeStep"];
  config.simTimeStep = configJson["simulation"]["timeStep"];
  config.hitRadius = configJson["simulation"]["hitRadius"];

  return true;
}

bool readAmmoJson(AmmoParams*& ammoList, int& ammoCount) {
  std::ifstream ammoFile("ammo.json");
  if (!ammoFile) {
    ERROR_LOG("Не вдалося відкрити файл \"ammo.json\"");
    return false;
  }

  json ammoJson;
  ammoFile >> ammoJson;

  ammoCount = static_cast<int>(ammoJson.size());
  if (ammoCount <= 0) {
    ERROR_LOG("Файл \"ammo.json\" не містить боєприпасів");
    return false;
  }

  ammoList = new AmmoParams[ammoCount]{};
  for (int i = 0; i < ammoCount; ++i) {
    std::strncpy(ammoList[i].name, ammoJson[i]["name"].get<std::string>().c_str(), sizeof(ammoList[i].name) - 1);
    ammoList[i].name[sizeof(ammoList[i].name) - 1] = '\0';

    ammoList[i].mass = ammoJson[i]["mass"];
    ammoList[i].drag = ammoJson[i]["drag"];
    ammoList[i].lift = ammoJson[i]["lift"];
  }

  return true;
}

bool writeSimulationJson(const SimStep steps[], int stepCount) {
  std::ofstream outputFile("simulation.json");
  if (!outputFile) {
    ERROR_LOG("Не вдалося відкрити файл \"simulation.json\" для запису!");
    return false;
  }

  nlohmann::ordered_json outputJson;
  outputJson["totalSteps"] = stepCount - 1;
  outputJson["steps"] = nlohmann::ordered_json::array();

  for (int i = 0; i < stepCount; ++i) {
    nlohmann::ordered_json stepJson;
    stepJson["position"] = {
      {"x", steps[i].pos.x},
      {"y", steps[i].pos.y}
    };
    stepJson["direction"] = steps[i].direction;
    stepJson["state"] = steps[i].state;
    stepJson["targetIndex"] = steps[i].targetIdx;
    stepJson["dropPoint"] = {
      {"x", steps[i].dropPoint.x},
      {"y", steps[i].dropPoint.y}
    };
    stepJson["aimPoint"] = {
      {"x", steps[i].aimPoint.x},
      {"y", steps[i].aimPoint.y}
    };
    stepJson["predictedTarget"] = {
      {"x", steps[i].predictedTarget.x},
      {"y", steps[i].predictedTarget.y}
    };

    outputJson["steps"].push_back(stepJson);
  }

  outputFile << outputJson.dump(2);
  return true;
}

void freeAmmo(AmmoParams*& ammoList) {
  delete[] ammoList;
  ammoList = nullptr;
}

void freeSteps(SimStep*& steps) {
  delete[] steps;
  steps = nullptr;
}


bool readTargetsJson(TargetData& targets) {
  std::ifstream targetsFile("targets.json");
  if (!targetsFile) {
    ERROR_LOG("Не вдалося відкрити файл \"targets.json\"");
    return false;
  }

  json targetsJson;
  targetsFile >> targetsJson;

  targets.targetCount = targetsJson["targetCount"];
  targets.timeSteps = targetsJson["timeSteps"];
  if (targets.targetCount <= 0 || targets.timeSteps <= 0) {
    ERROR_LOG("Файл \"targets.json\" містить некоректні розміри масиву цілей");
    return false;
  }

  targets.positions = new Coord*[targets.targetCount]{};
  for (int i = 0; i < targets.targetCount; ++i) {
    targets.positions[i] = new Coord[targets.timeSteps]{};
    for (int j = 0; j < targets.timeSteps; ++j) {
      targets.positions[i][j].x = targetsJson["targets"][i]["positions"][j]["x"];
      targets.positions[i][j].y = targetsJson["targets"][i]["positions"][j]["y"];
    }
  }

  return true;
}

void freeTargets(TargetData& targets) {
  if (targets.positions != nullptr) {
    for (int i = 0; i < targets.targetCount; ++i) {
      delete[] targets.positions[i];
    }
    delete[] targets.positions;
  }

  targets.positions = nullptr;
  targets.targetCount = 0;
  targets.timeSteps = 0;
}



bool interpolateTargetPosition(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep, Coord& targetPos) {
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

bool calculateTargetVelocity(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep, double& targetVx, double& targetVy) {
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

  Coord velocity = (secondPos - firstPos) / dt;
  targetVx = velocity.x;
  targetVy = velocity.y;

  return true;
}

bool predictTargetPosition(Coord& predictedPos, const TargetData& targets, int targetIndex, double simulationTime, double arrayTimeStep, double timeToImpact) {
  Coord currentPos{};
  if (!interpolateTargetPosition(targetIndex, targets, simulationTime, arrayTimeStep, currentPos)) {
    return false;
  }
  double targetVx, targetVy;
  if (!calculateTargetVelocity(targetIndex, targets, simulationTime, arrayTimeStep, targetVx, targetVy)) {
    return false;
  }
  if (timeToImpact < 0) {
    ERROR_LOG("Загальний час не може бути меншим за нуль!");
    return false;
  }
  predictedPos = currentPos + Coord{targetVx, targetVy} * timeToImpact;

  return true;
}

void calculateDropPoint(Coord& dropPoint, Coord& maneuverPoint, bool& needManeuver, const Coord& targetPos, const Coord& dronePos, double horizontalDistance, double accelerationPath) {
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

double calculateTimeToStop(DronePhase phase, double currentSpeed, double acceleration, double turnRemainingTime) {
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

double estimateTravelTime(double distance, double startSpeed, double attackSpeed, double acceleration) {
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

double calculateAngleDifference(double currentDir, double targetDir) {
  double delta = targetDir - currentDir;

  if (delta > M_PI) {
    delta -= 2.0 * M_PI;
  }
  if (delta < -M_PI) {
    delta += 2.0 * M_PI;
  }

  return delta;
}

bool estimateTimeToCompleteMove(double& totalTime, double& currentSpeed, double& currentDir, const Coord& startPos, const Coord& goalPos, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold) {
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

bool estimateTimeToTargetPath(double& totalTime, const Coord& currentPos, double currentSpeed, double currentDir, DronePhase currentPhase, int currentTargetIndex, int targetIndex, double turnTargetDir, double turnRemainingTime, bool needManeuver, const Coord& maneuverPoint, const Coord& dropPoint, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold) {
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

bool evaluateTarget(BestTargetResult& best, const DroneConfig& config, const DroneMotionState& droneMotion, const TargetData& targets, int targetIndex, const Coord& dronePosition, double simulationTime, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver) {
  Coord currentTargetPos{};
  if (!interpolateTargetPosition(targetIndex, targets, simulationTime, config.arrayTimeStep, currentTargetPos)) {
    ERROR_LOG("Невірний розрахунок позиції цілі");
    return false;
  }
  Coord predictedTargetPos = currentTargetPos;
  double totalTime = 0.0;
  Coord finalDropPoint{};
  bool finalNeedManeuver = false;
  Coord finalManeuverPoint{};

  const int iterations = 3;
  for (int iter = 0; iter < iterations; ++iter) {
    Coord currentDropPoint{};
    Coord currentManeuverPoint{};
    bool currentNeedManeuver = false;

    calculateDropPoint(currentDropPoint, currentManeuverPoint, currentNeedManeuver, predictedTargetPos, dronePosition, ballistics.horizontalDistance, config.accelPath);

    if (returningFromManuver && targetIndex == droneMotion.currentTargetIndex) {
      currentNeedManeuver = false;
    }

    if (!estimateTimeToTargetPath(totalTime, dronePosition, droneMotion.currentSpeed, droneMotion.currentDir, droneMotion.phase, droneMotion.currentTargetIndex, targetIndex, droneMotion.turnTargetDir, droneMotion.turnRemainingTime, currentNeedManeuver, currentManeuverPoint, currentDropPoint, config.attackSpeed, acceleration, config.angularSpeed, config.turnThreshold)) {
      ERROR_LOG("Невірна оцінка часу до точки скиду");
      return false;
    }

    finalDropPoint = currentDropPoint;
    finalNeedManeuver = currentNeedManeuver;
    finalManeuverPoint = currentManeuverPoint;

    const double timeToImpact = totalTime + ballistics.flightTime;
    if (!predictTargetPosition(predictedTargetPos, targets, targetIndex, simulationTime, config.arrayTimeStep, timeToImpact)) {
      ERROR_LOG("Невірний прогноз позиції цілі");
      return false;
    }
  }

  if (!estimateTimeToTargetPath(totalTime, dronePosition, droneMotion.currentSpeed, droneMotion.currentDir, droneMotion.phase, droneMotion.currentTargetIndex, targetIndex, droneMotion.turnTargetDir, droneMotion.turnRemainingTime, finalNeedManeuver, finalManeuverPoint, finalDropPoint, config.attackSpeed, acceleration, config.angularSpeed, config.turnThreshold)) {
    ERROR_LOG("Невірна фінальна оцінка часу до точки скиду");
    return false;
  }

  best.targetIndex = targetIndex;
  best.dropPoint = finalDropPoint;
  best.needManeuver = finalNeedManeuver;
  best.maneuverPoint = finalManeuverPoint;
  best.totalTime = totalTime;
  best.targetPos = currentTargetPos;
  best.predictedTarget = predictedTargetPos;
  return true;
}

bool isInsideRadius(const Coord& point, const Coord& center, double radius) {
    return length(point - center) <= radius;
}

void moveDroneToStop(Coord& position, double direction, double distance) {
  position.x += cos(direction) * distance;
  position.y += sin(direction) * distance;
}

bool moveDroneToPoint(Coord& position, const Coord& dest, double maxDistance){
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

bool updateDroneMotion(Coord& dronePosition, DroneMotionState& droneMotion, const Coord& goal, double simTimeStep, double attackSpeed, double acceleration, double angularSpeed, double turnThreshold) {

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

bool selectBestTarget(BestTargetResult &result, const DroneConfig& config, const DroneMotionState& droneMotion, const Coord& dronePosition, const TargetData& targets, double simulationTime, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver) {
  result.targetIndex = -1;
  double minTotalTime = VERY_LARGE_TIME;
  double currentTargetTotalTime = VERY_LARGE_TIME;

  for (int targetIndex = 0; targetIndex < targets.targetCount; ++targetIndex) {
    BestTargetResult candidate;
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
      minTotalTime = currentTargetTotalTime;
    }
  }

  if (!interpolateTargetPosition(result.targetIndex, targets, simulationTime, config.arrayTimeStep, result.targetPos)) {
    ERROR_LOG("Невірний розрахунок позиції найкращої цілі");
    return false;
  }
  return true;
}

double calculateDroneAcceleration(double attackSpeed, double accelerationPath) {
  if (accelerationPath <= 0.0) {
    return 0.0;
  }
  return attackSpeed * attackSpeed / (2.0 * accelerationPath);
}

int main() {
  int step = 0;

  DroneConfig config;
  Coord dronePosition;
  AmmoParams* ammoList = nullptr;
  int ammoCount = 0;
  TargetData targets;
  double simulationTime = 0.0;
  double acceleration = 0.0;

  if (!readConfigJson(config)) {
    return 1;
  }

  LOG("Конфігурацію завантажено: швидкість=" << config.attackSpeed
      << ", шлях розгону=" << config.accelPath
      << ", боєприпас=" << config.ammoName
      << ", старт=(" << config.startPos.x << "," << config.startPos.y << ")");
  dronePosition.x = config.startPos.x;
  dronePosition.y = config.startPos.y;

  if (!readTargetsJson(targets)) {
    ERROR_LOG("Невірний формат даних у файлі \"targets.json\"");
    return 1;
  }
  LOG("Цілі завантажено: " << targets.targetCount << ", кроків часу: " << targets.timeSteps);

  if (config.altitude <= 0) {
    ERROR_LOG("Висота дрона повинна бути більшою за нуль.");
    freeTargets(targets);
    return 1;
  }

  if (config.attackSpeed <= 0) {
    ERROR_LOG("Швидкість атаки повинна бути більшою за нуль.");
    freeTargets(targets);
    return 1;
  }

  if (!readAmmoJson(ammoList, ammoCount)) {
    freeTargets(targets);
    return 1;
  }
  LOG("Боєприпаси завантажено: " << ammoCount);

  // Вибір боєприпасу
  const AmmoParams* ammo = ammoSelect(ammoList, ammoCount, config.ammoName);
  if (ammo == nullptr) {
    ERROR_LOG("Невідомий тип боєприпасу: " << config.ammoName);
    freeAmmo(ammoList);
    freeTargets(targets);
    return 1;
  }
  LOG("Боєприпас знайдено: " << ammo->name);

  BallisticsResult ballistics;
  if (!calculateBallistics(ballistics, *ammo, config.attackSpeed, config.altitude)) {
    ERROR_LOG("Неправильна умова розрахунку балістики");
    freeAmmo(ammoList);
    freeTargets(targets);
    return 1;
  }
  LOG("Балістику розраховано: час польоту=" << ballistics.flightTime
      << ", горизонтальна дистанція=" << ballistics.horizontalDistance);

  acceleration = calculateDroneAcceleration(config.attackSpeed, config.accelPath);
  if (acceleration <= 0.0) {
    ERROR_LOG("Неправильний розрахунок прискорення дрона");
    freeAmmo(ammoList);
    freeTargets(targets);
    return 1;
  }
  // Визначення найкращої ціді.
  BestTargetResult best;
  DroneMotionState droneMotion;
  droneMotion.currentSpeed = 0.0;
  droneMotion.currentDir = config.initialDir;
  droneMotion.turnTargetDir = config.initialDir;
  droneMotion.turnRemainingTime = 0.0;
  droneMotion.phase = STOPPED;
  if (!selectBestTarget(best, config, droneMotion, dronePosition, targets, simulationTime, ballistics, acceleration, false)) {
    ERROR_LOG("Не знайдено найкращої цілі");
    freeAmmo(ammoList);
    freeTargets(targets);
    return 1;
  }
  LOG("Початкову ціль обрано: #" << best.targetIndex
      << ", точка скиду=(" << best.dropPoint.x << "," << best.dropPoint.y << ")"
      << ", потрібен маневр=" << best.needManeuver);

  int maxSteps = MAX_STEPS;
  SimStep* steps = new SimStep[maxSteps]{};

  droneMotion.currentTargetIndex = best.targetIndex;
  steps[0].pos = dronePosition;
  steps[0].direction = droneMotion.currentDir;
  steps[0].state = droneMotion.phase;
  steps[0].targetIdx = best.targetIndex;
  steps[0].dropPoint = best.dropPoint;
  steps[0].predictedTarget = best.predictedTarget;
  Coord initialDir{cos(config.initialDir), sin(config.initialDir)};
  steps[0].aimPoint = dronePosition + initialDir * ballistics.horizontalDistance;
  step = 1;
  Coord goal{};

  bool returningFromManuver = false;
  int maneuverTargetIndex = -1;
  // захоплення цілі
  bool targetLocked = false;
  int lockedTargetIndex = -1;

  while (step < maxSteps) {
    // Тест захоплення цілі
    if (targetLocked) {
      if (!evaluateTarget(best, config, droneMotion, targets, lockedTargetIndex, dronePosition, simulationTime, ballistics, acceleration, returningFromManuver)) {
        ERROR_LOG("Не вдалося переоцінити зафіксовану ціль на кроці " << step);
        break;
      }

    best.targetIndex = lockedTargetIndex;

    if (!interpolateTargetPosition(best.targetIndex, targets, simulationTime, config.arrayTimeStep, best.targetPos)) {
        ERROR_LOG("Невірний розрахунок позиції зафіксованої цілі");
        break;
      }
    } else {
      if (!selectBestTarget(best, config, droneMotion, dronePosition, targets, simulationTime, ballistics, acceleration, returningFromManuver)) {
        ERROR_LOG("Не знайдено найкращої цілі на кроці " << step);
        break;
      }
    }


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
    }

    // Коли дрон піде на маневр, то він запамʼятовує ціль, що б не смикатись після нової оцінки цілей під час виконання маневру
    bool headingToManeuver = best.needManeuver && !atManeuverPoint;
    if (headingToManeuver) {
      goal = best.maneuverPoint;
      maneuverTargetIndex = best.targetIndex;
      
      targetLocked = true;
      lockedTargetIndex = best.targetIndex;
    } else {
      goal = best.dropPoint;
    }
    // Коли маневр не потрібен, дрон може переобирати цілі до виходу в зону атаки.
    double distanceToDropPoint = distanceBetween(dronePosition, best.dropPoint);
    if (!headingToManeuver && droneMotion.phase == MOVING && !targetLocked && distanceToDropPoint <= ballistics.horizontalDistance) {
      targetLocked = true;
      lockedTargetIndex = best.targetIndex;
    }
    updateDroneMotion(dronePosition, droneMotion, goal, config.simTimeStep, config.attackSpeed, acceleration, config.angularSpeed, config.turnThreshold);

    droneMotion.currentTargetIndex = best.targetIndex; // Перевірити, на переключення цілей

    steps[step].pos = dronePosition;
    steps[step].direction = droneMotion.currentDir;
    steps[step].targetIdx = best.targetIndex;
    steps[step].state = droneMotion.phase; // MOVING;
    steps[step].dropPoint = best.dropPoint;
    steps[step].predictedTarget = best.predictedTarget;
    Coord currentDir{cos(droneMotion.currentDir), sin(droneMotion.currentDir)};
    steps[step].aimPoint = dronePosition + currentDir * ballistics.horizontalDistance;
    DEBUG("Крок " << step
          << " позиція=(" << dronePosition.x << "," << dronePosition.y << ")"
          << " напрям=" << droneMotion.currentDir
          << " швидкість=" << droneMotion.currentSpeed
          << " стан=" << droneMotion.phase
          << " ціль=" << best.targetIndex
          << " точка_скиду=(" << best.dropPoint.x << "," << best.dropPoint.y << ")"
          << " маневр=(" << best.maneuverPoint.x << "," << best.maneuverPoint.y << ")"
          << " потрібен_маневр=" << best.needManeuver
          << " точка_падіння=(" << steps[step].aimPoint.x << "," << steps[step].aimPoint.y << ")");

    simulationTime += config.simTimeStep;
    step++;

    // Перевірка на умови скиду
    bool dropNow = !headingToManeuver && isInsideRadius(dronePosition, best.dropPoint, config.hitRadius) && droneMotion.phase == MOVING;
    
    

    if (dropNow) {
      // скинути захоплення цілі
      targetLocked = false;
      lockedTargetIndex = -1;
      LOG("Умову скиду виконано на кроці " << step - 1
        << ", ціль #" << best.targetIndex
        << ", дрон=(" << dronePosition.x << "," << dronePosition.y << ")"
        << ", точка скиду=(" << best.dropPoint.x << "," << best.dropPoint.y << ")");
      break;
    }
  }

  if (!writeSimulationJson(steps, step)) {
    freeSteps(steps);
    freeAmmo(ammoList);
    freeTargets(targets);
    return 1;
  }
  LOG("Симуляцію завершено. Кроків: " << step - 1);
  LOG("Файл \"simulation.json\" записано");

  freeSteps(steps);
  freeAmmo(ammoList);
  freeTargets(targets);
  return 0;
}
