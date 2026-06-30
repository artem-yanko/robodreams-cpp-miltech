#pragma once

#include <string>

struct AmmoParams {
    std::string name{};
    double mass{};
    double drag{};
    double lift{};
};

struct Coord {
    double x{};
    double y{};

    Coord operator+(const Coord& other) const { return {x + other.x, y + other.y}; }
    Coord operator-(const Coord& other) const { return {x - other.x, y - other.y}; }
    Coord operator*(double scalar) const { return {x * scalar, y * scalar}; }
    Coord operator/(double scalar) const { return {x / scalar, y / scalar}; }
    Coord& operator+=(const Coord& other) { x += other.x; y += other.y; return *this; }
};

struct DroneConfig {
    Coord startPos{};
    double altitude{};
    double initialDir{};
    double attackSpeed{};
    double accelPath{};
    std::string ammoName{};
    double arrayTimeStep{};
    double targetTimeStep{};
    double simTimeStep{};
    double physicsTimeStep{};
    double timeScale{};
    double hitRadius{};
    double angularSpeed{};
    double turnThreshold{};
};

struct BallisticsResult {
    double flightTime{};
    double horizontalDistance{};
};

struct DroneMotionState {
    double currentSpeed{};
    double currentDir{};
    double turnTargetDir{};
    double desiredDir{};
};
