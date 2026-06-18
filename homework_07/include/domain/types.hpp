#pragma once
#include <string>
#include <vector>

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
    bool operator==(const Coord& other) const { return x == other.x && y == other.y; }
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
    double timeScale{1.0};
    double hitRadius{};
    double angularSpeed{};
    double turnThreshold{};
};

struct SimStep {
    Coord pos{};
    double direction{};
    std::string state{};
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
    double desiredDir{};
    int currentTargetIndex{-1};
};

struct TargetData {
    int targetCount{};
    int timeSteps{};
    std::vector<std::vector<Coord>> positions{};
};

struct Target {
    Coord position{};
    Coord velocity{};
};

enum class DroneMode {
    Stopped,
    Accelerating,
    Decelerating,
    Turning,
    Moving
};

struct DroneCommand {
    DroneMode mode{DroneMode::Stopped};
    double angleSpeed{};
    double effectiveFromSimTime{};
};

struct DroneTelemetry {
    Coord pos{};
    Coord speed{};
    double direction{};
    DroneMode mode{DroneMode::Stopped};
};
