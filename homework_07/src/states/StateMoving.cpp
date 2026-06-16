#include "states/StateMoving.hpp"

#include "states/StateDecelerating.hpp"
#include "utils/math_utils.hpp"

#include <cmath>

static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.desiredDir);
    double slowTurnThreshold = ctx.config.turnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    if (std::fabs(angleLeft) > slowTurnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.command.mode = DroneMode::Decelerating;
        ctx.command.angleSpeed = 0.0;
        return std::make_unique<StateDecelerating>();
    }

    ctx.command.mode = DroneMode::Moving;
    if (std::fabs(angleLeft) > ctx.config.turnThreshold) {
        ctx.command.angleSpeed = angleLeft > 0.0 ? ctx.config.angularSpeed : -ctx.config.angularSpeed;
    } else {
        ctx.command.angleSpeed = 0.0;
    }

    return nullptr;
}

const char* StateMoving::name() const {
    return "Moving";
}
