#include "core/DronePhysics.hpp"

#include "utils/logger.hpp"

#include <chrono>
#include <cmath>
#include <thread>

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

static const char* modeName(DroneMode mode) {
    switch (mode) {
    case DroneMode::Stopped:
        return "Stopped";
    case DroneMode::Accelerating:
        return "Accelerating";
    case DroneMode::Decelerating:
        return "Decelerating";
    case DroneMode::Turning:
        return "Turning";
    case DroneMode::Moving:
        return "Moving";
    }

    return "Unknown";
}

DronePhysics::DronePhysics(const DroneConfig& config)
    : config_(config) {
    telemetry_.pos = config.startPos;
    telemetry_.direction = config.initialDir;
    telemetry_.mode = DroneMode::Stopped;
}

void DronePhysics::run() {
    threadReady_ = true;
    DEBUG("DronePhysics thread ready");

    while (running_ && !started_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    DEBUG("DronePhysics thread started");

    const double timeScale = config_.timeScale > 0.0 ? config_.timeScale : 1.0;
    auto lastTick = std::chrono::steady_clock::now();
    double accumulatedSimTime = 0.0;

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        accumulatedSimTime += std::chrono::duration<double>(now - lastTick).count() * timeScale;
        lastTick = now;

        while (running_ && accumulatedSimTime >= config_.physicsTimeStep) {
            step();
            accumulatedSimTime -= config_.physicsTimeStep;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    DEBUG("DronePhysics thread stopped");
}

void DronePhysics::start() {
    started_ = true;
    DEBUG("DronePhysics start signal received");
}

void DronePhysics::stop() {
    running_ = false;
    DEBUG("DronePhysics stop signal received");
}

bool DronePhysics::isThreadReady() const {
    return threadReady_;
}

void DronePhysics::submitCommand(const DroneCommand& command) {
    commandQueue_.push(command);
    DEBUG("DronePhysics command queued: mode=" << modeName(command.mode)
        << ", angleSpeed=" << command.angleSpeed);
}

DroneTelemetry DronePhysics::getTelemetry() const {
    std::lock_guard<std::mutex> lock(telemetryMutex_);
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

    std::lock_guard<std::mutex> lock(telemetryMutex_);
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
