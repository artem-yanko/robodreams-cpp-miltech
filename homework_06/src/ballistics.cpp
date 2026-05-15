#define _USE_MATH_DEFINES
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include "ballistics.hpp"

const double gravity = 9.81;

struct AmmoParams {
  char name[32]{};
  double mass;
  double drag;
  double lift;
};

const AmmoParams* ammoSelect(const AmmoParams* ammoList, int ammoCount, const char* ammoName)
{
  for (int i = 0; i < ammoCount; ++i) {
    if (strcmp(ammoName, ammoList[i].name) == 0) {
      return &ammoList[i];
    }
  }
  return nullptr;
}

const AmmoParams ammoList[5] = {{"VOG-17", 0.35, 0.07, 0.0},
                                {"M67", 0.6, 0.10, 0.0},
                                {"RKG-3", 1.2, 0.10, 0.0},
                                {"GLIDING-VOG", 0.45, 0.10, 1.0},
                                {"GLIDING-RKG", 1.4, 0.10, 1.0}};

void readFile(const char* filename, BallisticsInput& input)
{
  std::ifstream inputFile(filename);
  if (!inputFile) {
    std::cerr << "ERROR: Перевірте наявність файлу \"" << filename << "\"!" << std::endl;
    exit(1);
  }

  if (!(inputFile >> input.xd >> input.yd >> input.zd >> input.targetX >> input.targetY >> input.attackSpeed >> input.accelerationPath >>
        input.ammoName)) {
    std::cerr << "ERROR: Невірний формат даних у файлі \"" << filename << "\"!" << std::endl;
    exit(1);
  }
}

void writeFile(const char* filename, const BallisticsResult& result)
{
  std::ofstream outputFile(filename);
  if (!outputFile) {
    std::cerr << "ERROR: Не вдалося відкрити файл \"" << filename << "\" для запису!" << std::endl;
    exit(1);
  }

  if (result.needManeuver) {
    outputFile << result.maneuverX << " " << result.maneuverY << " ";
  }
  outputFile << result.fireX << " " << result.fireY << std::endl;
}

double calculateDistanceToTarger(double targetX, double targetY, double xd, double yd)
{
  // return sqrt(pow(targetX - xd, 2)) + pow(targetY - yd, 2);
  double dx = targetX - xd;
  double dy = targetY - yd;
  return sqrt(dx * dx + dy * dy);
}

bool calculateTimeOfFlight(const AmmoParams* ammo, double attackSpeed, double zd, double& time)
{
  double a = ammo->drag * gravity * ammo->mass - 2 * ammo->drag * ammo->drag * ammo->lift * attackSpeed;
  double b = -3 * gravity * ammo->mass * ammo->mass + 3 * ammo->drag * ammo->lift * ammo->mass * attackSpeed;
  double c = 6 * ammo->mass * ammo->mass * zd;

  if (a == 0) {
    return false;
  }

  double p = -((b * b) / (3 * a * a));
  double q = (2 * b * b * b) / (27 * a * a * a) + c / a;

  if (p == 0) {
    return false;
  }

  double acosArg1 = ((3 * q) / (2 * p));

  if ((-3 / p) < 0) {
    return false;
  }

  double acosArg2 = sqrt(-3 / p);

  double acosArg = acosArg1 * acosArg2;
  if (acosArg > 1 || acosArg < -1) {
    std::cerr << "ERROR: Невірне значення для арккосинуса: " << acosArg << std::endl;
    return false;
  }

  double phi = acos(acosArg);

  time = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);
  if (time <= 0) {
    std::cerr << "ERROR: невірний розрахунок часу польоту: " << time << std::endl;
    return false;
  }

  return true;
}

double calculateHorizontalDistance(const AmmoParams* ammo, double attackSpeed, double time)
{
  double time2 = time * time;
  double time3 = time2 * time;
  double time4 = time3 * time;
  double time5 = time4 * time;

  double drag2 = ammo->drag * ammo->drag;
  double drag3 = drag2 * ammo->drag;
  double drag4 = drag3 * ammo->drag;

  double lift2 = ammo->lift * ammo->lift;
  double lift3 = lift2 * ammo->lift;
  double lift4 = lift3 * ammo->lift;

  double mass2 = ammo->mass * ammo->mass;
  double mass3 = mass2 * ammo->mass;
  double mass4 = mass3 * ammo->mass;

  // V₀t − t²d·V₀/(2m)
  double part1 = attackSpeed * time - ((time2 * (ammo->drag * attackSpeed))) / (2.0 * ammo->mass);
  // t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²)
  double part2 =
    (time3 * (6.0 * ammo->drag * gravity * ammo->lift * ammo->mass - 6.0 * drag2 * (lift2 - 1) * attackSpeed)) / (36.0 * mass2);
  // t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀) / (36(1+l²)²m³)
  double part3 =
    (time4 * ((-6.0 * drag2 * gravity * ammo->lift * (1 + ammo->lift * ammo->lift + ammo->lift * ammo->lift * ammo->lift * ammo->lift)) *
                ammo->mass +
              (3.0 * drag3 * ammo->lift * ammo->lift * (1 + ammo->lift * ammo->lift) * attackSpeed) +
              (6.0 * drag3 * ammo->lift * ammo->lift * ammo->lift * ammo->lift * (1 + ammo->lift * ammo->lift) * attackSpeed))) /
    (36.0 * pow(1.0 + ammo->lift * ammo->lift, 2) * ammo->mass * ammo->mass * ammo->mass);
  // t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
  double part4 = (time5 * (3.0 * drag3 * gravity * ammo->lift * ammo->lift * ammo->lift * ammo->mass -
                           3.0 * drag4 * ammo->lift * ammo->lift * (1 + ammo->lift * ammo->lift) * attackSpeed)) /
                 (36.0 * (1.0 + ammo->lift * ammo->lift) * ammo->mass * ammo->mass * ammo->mass);

  return part1 + part2 + part3 + part4;
}

bool calculateBallitsics(const BallisticsInput& input, BallisticsResult& result)
{
  if (input.zd <= 0) {
    std::cerr << "ERROR: Висота дрона повинна бути більшою за нуль." << std::endl;
    return false;
  }

  if (input.attackSpeed <= 0) {
    std::cerr << "ERROR: Швидкість атаки повинна бути більшою за нуль." << std::endl;
    return false;
  }

  double initX = input.xd;
  double initY = input.yd;
  double maneuverX = 0.0;
  double maneuverY = 0.0;
  double flightTime = 0.0;
  const int ammoCount = 5;
  const AmmoParams* selectedAmmo = ammoSelect(ammoList, ammoCount, input.ammoName);
  if (selectedAmmo == nullptr) {
    std::cerr << "ERROR: Невідомий тип боєприпасу: " << input.ammoName << std::endl;
    return false;
  }
  if (!(calculateTimeOfFlight(selectedAmmo, input.attackSpeed, input.zd, flightTime))) {
    std::cerr << "ERROR: Неправильна умова розрахунку часу польоту" << std::endl;
    return false;
  }
  double horizontalDistance = calculateHorizontalDistance(selectedAmmo, input.attackSpeed, flightTime);
  double distanceToTarget = calculateDistanceToTarger(input.targetX, input.targetY, initX, initY);
  if (distanceToTarget <= 0) {
    std::cerr << "ERROR: Дистанція дрон-ціль має бути більше за 0. Поточна дистанція: " << distanceToTarget << std::endl;
    return false;
  }

  // Якщо є потреба маневру - обрахунок координат точки для маневру
  bool needManeuver = (distanceToTarget > 0) && (horizontalDistance + input.accelerationPath > distanceToTarget);
  if (needManeuver) {
    maneuverX = input.targetX - (input.targetX - initX) * (horizontalDistance + input.accelerationPath) / distanceToTarget;
    maneuverY = input.targetY - (input.targetY - initY) * (horizontalDistance + input.accelerationPath) / distanceToTarget;
    initX = maneuverX;
    initY = maneuverY;
    distanceToTarget = calculateDistanceToTarger(input.targetX, input.targetY, initX, initY);
  }

  double fireX = 0.0;
  double fireY = 0.0;

  // Обрахунок точки скиду
  if (distanceToTarget > 0) {
    double ratio = (distanceToTarget - horizontalDistance) / distanceToTarget;
    fireX = initX + (input.targetX - initX) * ratio;
    fireY = initY + (input.targetY - initY) * ratio;
  }

  result.needManeuver = needManeuver;
  result.maneuverX = maneuverX;
  result.maneuverY = maneuverY;
  result.fireX = fireX;
  result.fireY = fireY;

  return true;
}