#include "states/StateStopped.hpp"
#include "states/StateTurning.hpp"
#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx) {
    double deltaAngle = std::fabs(calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.desiredDir));
    if (deltaAngle > ctx.config.turnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.command.mode = DroneMode::Turning;
        ctx.command.angleSpeed =
            calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.desiredDir) > 0.0
                ? ctx.config.angularSpeed
                : -ctx.config.angularSpeed;
        return std::make_unique<StateTurning>();
    }

    ctx.command.mode = DroneMode::Accelerating;
    ctx.command.angleSpeed = 0.0;
    return std::make_unique<StateAccelerating>();
}

const char* StateStopped::name() const {
    return "Stopped";
}
