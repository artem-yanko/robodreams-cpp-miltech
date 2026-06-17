#pragma once

#include "core/ThreadSafeQueue.hpp"
#include "domain/types.hpp"
#include <atomic>
#include <mutex>

class DronePhysics {
public:
    explicit DronePhysics(const DroneConfig& config);

    void run();
    void start();
    void stop();
    bool isThreadReady() const;

    void submitCommand(const DroneCommand& command);
    DroneTelemetry getTelemetry() const;
    void step();

private:
    DroneConfig config_{};
    ThreadSafeQueue<DroneCommand> commandQueue_{};
    DroneCommand currentCommand_{};
    mutable std::mutex telemetryMutex_{};
    DroneTelemetry telemetry_{};
    std::atomic<bool> running_{true};
    std::atomic<bool> started_{false};
    std::atomic<bool> threadReady_{false};
};
