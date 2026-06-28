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
