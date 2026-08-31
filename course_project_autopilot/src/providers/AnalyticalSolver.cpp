#include "providers/AnalyticalSolver.hpp"

#include "utils/logger.hpp"

#include <cmath>

static const double GRAVITY = 9.81;

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

    double part1 = attackSpeed * time - ((time2 * (drag * attackSpeed))) / (2.0 * mass);
    double part2 = (time3 * (6.0 * drag * GRAVITY * lift * mass - 6.0 * drag2 * (lift2 - 1) * attackSpeed))
        / (36.0 * mass2);
    double part3 = (time4 * ((-6.0 * drag2 * GRAVITY * lift * (1 + lift2 + lift4)) * mass
        + (3.0 * drag3 * lift2 * (1 + lift2) * attackSpeed)
        + (6.0 * drag3 * lift4 * (1 + lift2) * attackSpeed)))
        / (36.0 * std::pow(1.0 + lift2, 2) * mass3);
    double part4 = (time5 * (3.0 * drag3 * GRAVITY * lift3 * mass - 3.0 * drag4 * lift2 * (1 + lift2) * attackSpeed))
        / (36.0 * (1.0 + lift2) * mass4);

    return part1 + part2 + part3 + part4;
}

static bool calculateTimeOfFlight(const AmmoParams& ammo, double attackSpeed, double altitude, double& flightTime) {
    const double mass = ammo.mass;
    const double drag = ammo.drag;
    const double lift = ammo.lift;

    double a = drag * GRAVITY * mass - 2 * drag * drag * lift * attackSpeed;
    double b = -3 * GRAVITY * mass * mass + 3 * drag * lift * mass * attackSpeed;
    double c = 6 * mass * mass * altitude;

    if (a == 0.0) {
        return false;
    }

    double p = -((b * b) / (3 * a * a));
    double q = (2 * b * b * b) / (27 * a * a * a) + c / a;
    if (p == 0.0) {
        return false;
    }

    double acosArg1 = (3 * q) / (2 * p);
    if ((-3 / p) < 0) {
        return false;
    }

    double acosArg2 = std::sqrt(-3 / p);
    double acosArg = acosArg1 * acosArg2;
    if (acosArg > 1.0 || acosArg < -1.0) {
        ERROR_LOG("Invalid value for arccos: " << acosArg);
        return false;
    }

    double phi = std::acos(acosArg);
    flightTime = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * M_PI) / 3) - b / (3 * a);
    if (flightTime <= 0.0) {
        ERROR_LOG("Invalid flight time calculation: " << flightTime);
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

BallisticsResult AnalyticalSolver::solve(const DroneConfig& config, const AmmoParams& ammo) {
    BallisticsResult result{};
    if (!calculateBallistics(result, ammo, config.attackSpeed, config.altitude)) {
        ERROR_LOG("Invalid ballistic calculation conditions");
        return result;
    }

    LOG("Analytical ballistics calculated: flightTime=" << result.flightTime
        << ", horizontalDistance=" << result.horizontalDistance);
    return result;
}
