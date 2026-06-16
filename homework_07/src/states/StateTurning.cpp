#include "states/StateTurning.hpp"
#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.turnTargetDir);

    if (std::fabs(angleLeft) <= ctx.config.turnThreshold) {
        ctx.command.mode = DroneMode::Accelerating;
        ctx.command.angleSpeed = 0.0;
        return std::make_unique<StateAccelerating>();
    }

    ctx.command.mode = DroneMode::Turning;
    ctx.command.angleSpeed = angleLeft > 0.0 ? ctx.config.angularSpeed : -ctx.config.angularSpeed;
    return nullptr;
}

const char* StateTurning::name() const {
    return "Turning";
}
