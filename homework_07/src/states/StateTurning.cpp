#include "states/StateTurning.hpp"
#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.turnTargetDir);

    if (std::fabs(angleLeft) <= ctx.activeTurnThreshold) {
        ctx.command.mode = DroneMode::Accelerating;
        ctx.command.angleSpeed = 0.0;
        return std::make_unique<StateAccelerating>();
    }

    ctx.command.mode = DroneMode::Turning;
    double turnStep = ctx.config.physicsTimeStep > 0.0 ? ctx.config.physicsTimeStep : ctx.config.simTimeStep;
    double requiredAngleSpeed = turnStep > 0.0 ? angleLeft / turnStep : 0.0;
    if (requiredAngleSpeed > ctx.config.angularSpeed) {
        requiredAngleSpeed = ctx.config.angularSpeed;
    } else if (requiredAngleSpeed < -ctx.config.angularSpeed) {
        requiredAngleSpeed = -ctx.config.angularSpeed;
    }
    ctx.command.angleSpeed = requiredAngleSpeed;
    return nullptr;
}

const char* StateTurning::name() const {
    return "Turning";
}
