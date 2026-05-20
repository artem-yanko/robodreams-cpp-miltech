#include "providers/AnalyticalSolver.hpp"
#include "utils/logging.hpp"
#include <cmath>

const double gravity = 9.81;


static double calculateHorizontalDistance(const AmmoParams& ammo, double attackSpeed, double time) {
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

static bool calculateTimeOfFlight(const AmmoParams& ammo, double attackSpeed, double zd, double& flightTime) {
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

static bool calculateBallistics(BallisticsResult& result, const AmmoParams& ammo, double attackSpeed, double altitude) {
  result.flightTime = 0.0;
  result.horizontalDistance = 0.0;

  if (!calculateTimeOfFlight(ammo, attackSpeed, altitude, result.flightTime)) {
    return false;
  }

  result.horizontalDistance = calculateHorizontalDistance(ammo, attackSpeed, result.flightTime);
  return true;
}

BallisticsResult AnalyticalSolver::solve(const Coord& dronePos, const Target& target, double altitude, double attackSpeed, const AmmoParams& ammo) {
    (void)dronePos;
    (void)target;
    BallisticsResult result{};
    if (!calculateBallistics(result, ammo, attackSpeed, altitude)) {
        ERROR_LOG("Неправильна умова розрахунку балістики");
        return result;
    }
    LOG("Балістику розраховано: час польоту=" << result.flightTime
      << ", горизонтальна дистанція=" << result.horizontalDistance);
    return result;
    }
