#include "states/StateDecelerating.hpp"
#include "states/StateTurning.hpp"
#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

static double speedLength(const Coord& speed) {
    return std::sqrt(speed.x * speed.x + speed.y * speed.y);
}

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx) {
    double currentSpeed = speedLength(ctx.telemetry.speed);
    if (currentSpeed <= 0.0) {
        double angleLeft = calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.desiredDir);
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        if (std::fabs(angleLeft) > ctx.activeTurnThreshold) {
            ctx.command.mode = DroneMode::Turning;
            ctx.command.angleSpeed = angleLeft > 0.0 ? ctx.config.angularSpeed : -ctx.config.angularSpeed;
            return std::make_unique<StateTurning>();
        }

        ctx.command.mode = DroneMode::Accelerating;
        ctx.command.angleSpeed = 0.0;
        return std::make_unique<StateAccelerating>();
    }

    ctx.command.mode = DroneMode::Decelerating;
    ctx.command.angleSpeed = 0.0;
    return nullptr;
}

const char* StateDecelerating::name() const {
    return "Decelerating";
}
