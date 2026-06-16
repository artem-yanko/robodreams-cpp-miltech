#include "core/DronePhysics.hpp"

#include <cmath>

static double vectorLength(const Coord& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

static double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

static Coord directionToVector(double direction, double speed) {
    return {std::cos(direction) * speed, std::sin(direction) * speed};
}

DronePhysics::DronePhysics(const DroneConfig& config)
    : config_(config) {
    telemetry_.pos = config.startPos;
    telemetry_.direction = config.initialDir;
    telemetry_.mode = DroneMode::Stopped;
}

void DronePhysics::submitCommand(const DroneCommand& command) {
    commandQueue_.push(command);
}

DroneTelemetry DronePhysics::getTelemetry() const {
    return telemetry_;
}

void DronePhysics::step() {
    DroneCommand nextCommand{};
    while (commandQueue_.tryPop(nextCommand)) {
        currentCommand_ = nextCommand;
    }

    telemetry_.mode = currentCommand_.mode;

    const double dt = config_.physicsTimeStep;
    const double acceleration = config_.accelPath > 0.0
        ? (config_.attackSpeed * config_.attackSpeed) / (2.0 * config_.accelPath)
        : 0.0;

    double currentSpeed = vectorLength(telemetry_.speed);

    switch (currentCommand_.mode) {
    case DroneMode::Stopped:
        telemetry_.speed = {0.0, 0.0};
        break;

    case DroneMode::Turning:
        telemetry_.direction = normalizeAngle(telemetry_.direction + currentCommand_.angleSpeed * dt);
        telemetry_.speed = {0.0, 0.0};
        break;

    case DroneMode::Accelerating:
        currentSpeed += acceleration * dt;
        if (currentSpeed > config_.attackSpeed) {
            currentSpeed = config_.attackSpeed;
        }
        telemetry_.speed = directionToVector(telemetry_.direction, currentSpeed);
        telemetry_.pos += telemetry_.speed * dt;
        break;

    case DroneMode::Moving:
        telemetry_.direction = normalizeAngle(telemetry_.direction + currentCommand_.angleSpeed * dt);
        currentSpeed = config_.attackSpeed;
        telemetry_.speed = directionToVector(telemetry_.direction, currentSpeed);
        telemetry_.pos += telemetry_.speed * dt;
        break;

    case DroneMode::Decelerating:
        currentSpeed -= acceleration * dt;
        if (currentSpeed < 0.0) {
            currentSpeed = 0.0;
        }
        telemetry_.speed = directionToVector(telemetry_.direction, currentSpeed);
        telemetry_.pos += telemetry_.speed * dt;
        break;
    }
}
