#pragma once

#include "domain/types.hpp"
#include <memory>

struct DroneContext {
    Coord& position;
    DroneMotionState& droneMotion;
    const Coord& goal;
    const DroneConfig& config;
    double acceleration;
    bool reachedGoal{false};
};

class IDroneState {
public:
    virtual ~IDroneState() = default;
    virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;
    virtual const char* name() const = 0;
};