#include "states/StateTurning.hpp"

#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"

#include <cmath>

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.dir, ctx.droneMotion.turnTargetDir);

    if (std::fabs(angleLeft) <= ctx.activeTurnThreshold) {
        ctx.decision.accel = 1.0f;
        ctx.decision.turnRate = 0.0f;
        return std::make_unique<StateAccelerating>();
    }

    double turnStep = ctx.config.physicsTimeStep > 0.0 ? ctx.config.physicsTimeStep : ctx.config.simTimeStep;
    double requiredAngleSpeed = turnStep > 0.0 ? angleLeft / turnStep : 0.0;
    if (requiredAngleSpeed > ctx.config.angularSpeed) {
        requiredAngleSpeed = ctx.config.angularSpeed;
    } else if (requiredAngleSpeed < -ctx.config.angularSpeed) {
        requiredAngleSpeed = -ctx.config.angularSpeed;
    }

    ctx.decision.accel = 0.0f;
    ctx.decision.turnRate = ctx.config.angularSpeed > 0.0
        ? static_cast<float>(requiredAngleSpeed / ctx.config.angularSpeed)
        : 0.0f;
    return nullptr;
}

const char* StateTurning::name() const {
    return "Turning";
}
