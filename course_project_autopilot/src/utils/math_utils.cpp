#include "utils/math_utils.hpp"

#include <cmath>

double length(const Coord& c) {
    return hypot(c.x, c.y);
}

double distanceBetween(const Coord& from, const Coord& to) {
    return length(to - from);
}

Coord normalize(const Coord& c) {
    double len = length(c);
    if (len == 0.0) {
        return {};
    }

    return c / len;
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

double calculateDroneAcceleration(double attackSpeed, double accelerationPath) {
    if (accelerationPath <= 0.0) {
        return 0.0;
    }

    return attackSpeed * attackSpeed / (2.0 * accelerationPath);
}
