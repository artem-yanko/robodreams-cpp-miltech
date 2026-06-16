#pragma once

#include "core/ThreadSafeQueue.hpp"
#include "domain/types.hpp"

class DronePhysics {
public:
    explicit DronePhysics(const DroneConfig& config);

    void submitCommand(const DroneCommand& command);
    DroneTelemetry getTelemetry() const;
    void step();

private:
    DroneConfig config_{};
    ThreadSafeQueue<DroneCommand> commandQueue_{};
    DroneCommand currentCommand_{};
    DroneTelemetry telemetry_{};
};
