#pragma once

#include "core/ThreadSafeQueue.hpp"
#include "domain/types.hpp"
#include <atomic>
#include <deque>
#include <mutex>

class DronePhysics {
public:
    explicit DronePhysics(const DroneConfig& config);

    void run();
    void start();
    void stop();
    bool isThreadReady() const;
    double getCurrentSimTime() const;
    void setAllowedSimTime(double simTime);

    void submitCommand(const DroneCommand& command);
    DroneTelemetry getTelemetry() const;
    DroneTelemetry getTelemetryAt(double simTime) const;
    void step();

private:
    struct TelemetrySnapshot {
        double simTime{};
        DroneTelemetry telemetry{};
    };

    DroneConfig config_{};
    ThreadSafeQueue<DroneCommand> commandQueue_{};
    DroneCommand currentCommand_{};
    std::deque<DroneCommand> pendingCommands_{};
    mutable std::mutex telemetryMutex_{};
    DroneTelemetry telemetry_{};
    double currentSimTime_{0.0};
    std::deque<TelemetrySnapshot> telemetryHistory_{};
    std::atomic<bool> running_{true};
    std::atomic<bool> started_{false};
    std::atomic<bool> threadReady_{false};
    std::atomic<double> allowedSimTime_{0.0};
};
