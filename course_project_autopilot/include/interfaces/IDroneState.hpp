#pragma once

#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"
#include "domain/types.hpp"
#include <memory>

struct DroneContext {
    const dlink::Telemetry& telemetry;
    DroneMotionState& droneMotion;
    const Coord& goal;
    const DroneConfig& config;
    double activeTurnThreshold;
    MissionDecision& decision;
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
