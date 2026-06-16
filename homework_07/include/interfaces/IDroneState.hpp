#pragma once

#include "domain/types.hpp"
#include <memory>

struct DroneContext {
    const DroneTelemetry& telemetry;
    DroneMotionState& droneMotion;
    const Coord& goal;
    const DroneConfig& config;
    DroneCommand& command;
};

class IDroneState {
public:
    virtual ~IDroneState() = default;
    virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;
    virtual const char* name() const = 0;
    virtual bool isMoving() const { return false; }
    virtual bool isDecelerating() const { return false; }
    virtual bool isAccelerating() const { return false; }
    virtual bool isTurning() const { return false; }
};
