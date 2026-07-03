#include "core/DronePhysics.hpp"

#include "utils/math_utils.hpp"
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

static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

static Coord lerpCoord(const Coord& a, const Coord& b, double t) {
    return {lerp(a.x, b.x, t), lerp(a.y, b.y, t)};
}

static constexpr std::size_t kTelemetryHistoryLimit = 256;

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
    telemetry_.speed = {};
    telemetry_.direction = config.initialDir;
    telemetry_.mode = DroneMode::Stopped;
    telemetryHistory_.push_back({0.0, telemetry_});
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

        while (running_
            && accumulatedSimTime >= config_.physicsTimeStep
            && currentSimTime_ + config_.physicsTimeStep <= allowedSimTime_) {
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

double DronePhysics::getCurrentSimTime() const {
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    return currentSimTime_;
}

void DronePhysics::setAllowedSimTime(double simTime) {
    allowedSimTime_ = simTime;
}

void DronePhysics::submitCommand(const DroneCommand& command) {
    commandQueue_.push(command);
    DEBUG("DronePhysics command queued: mode=" << modeName(command.mode)
        << ", angleSpeed=" << command.angleSpeed
        << ", effectiveFromSimTime=" << command.effectiveFromSimTime);
}

DroneTelemetry DronePhysics::getTelemetry() const {
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    return telemetry_;
}

DroneTelemetry DronePhysics::getTelemetryAt(double simTime) const {
    std::lock_guard<std::mutex> lock(telemetryMutex_);

    if (telemetryHistory_.empty()) {
        return {};
    }

    if (simTime <= telemetryHistory_.front().simTime) {
        return telemetryHistory_.front().telemetry;
    }

    if (simTime >= telemetryHistory_.back().simTime) {
        return telemetryHistory_.back().telemetry;
    }

    for (std::size_t i = 1; i < telemetryHistory_.size(); ++i) {
        const TelemetrySnapshot& previous = telemetryHistory_[i - 1];
        const TelemetrySnapshot& current = telemetryHistory_[i];
        if (simTime > current.simTime) {
            continue;
        }

        double fraction = (simTime - previous.simTime) / (current.simTime - previous.simTime);
        DroneTelemetry result{};
        result.pos = lerpCoord(previous.telemetry.pos, current.telemetry.pos, fraction);
        result.speed = lerpCoord(previous.telemetry.speed, current.telemetry.speed, fraction);
        result.direction = previous.telemetry.direction
            + calculateAngleDifference(previous.telemetry.direction, current.telemetry.direction) * fraction;
        result.mode = fraction < 0.5 ? previous.telemetry.mode : current.telemetry.mode;
        return result;
    }

    return telemetryHistory_.back().telemetry;
}

void DronePhysics::step() {
    DroneCommand nextCommand{};
    while (commandQueue_.tryPop(nextCommand)) {
        pendingCommands_.push_back(nextCommand);
    }

    const double dt = config_.physicsTimeStep;
    const double acceleration = config_.accelPath > 0.0
        ? (config_.attackSpeed * config_.attackSpeed) / (2.0 * config_.accelPath)
        : 0.0;

    std::lock_guard<std::mutex> lock(telemetryMutex_);
    while (!pendingCommands_.empty()
        && currentSimTime_ >= pendingCommands_.front().effectiveFromSimTime) {
        currentCommand_ = pendingCommands_.front();
        pendingCommands_.pop_front();
    }
    telemetry_.mode = currentCommand_.mode;
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

    currentSimTime_ += dt;
    telemetryHistory_.push_back({currentSimTime_, telemetry_});
    while (telemetryHistory_.size() > kTelemetryHistoryLimit) {
        telemetryHistory_.pop_front();
    }
}
