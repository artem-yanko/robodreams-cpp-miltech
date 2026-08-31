#include "states/StateMoving.hpp"

#include "states/StateDecelerating.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>

static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;
static const double ANGLE_EPSILON = 1e-6;

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.dir, ctx.droneMotion.desiredDir);
    double slowTurnThreshold = ctx.activeTurnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    if (std::fabs(angleLeft) > slowTurnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.decision.accel = -1.0f;
        ctx.decision.turnRate = 0.0f;
        return std::make_unique<StateDecelerating>();
    }

    if (std::fabs(angleLeft) > ANGLE_EPSILON) {
        double turnStep = ctx.config.physicsTimeStep > 0.0 ? ctx.config.physicsTimeStep : ctx.config.simTimeStep;
        double requiredAngleSpeed = turnStep > 0.0 ? angleLeft / turnStep : 0.0;
        if (requiredAngleSpeed > ctx.config.angularSpeed) {
            requiredAngleSpeed = ctx.config.angularSpeed;
        } else if (requiredAngleSpeed < -ctx.config.angularSpeed) {
            requiredAngleSpeed = -ctx.config.angularSpeed;
        }

        ctx.decision.turnRate = ctx.config.angularSpeed > 0.0
            ? static_cast<float>(requiredAngleSpeed / ctx.config.angularSpeed)
            : 0.0f;
    } else {
        ctx.decision.turnRate = 0.0f;
    }

    double speedError = ctx.config.attackSpeed - ctx.telemetry.speed;
    double normalizedAccel = ctx.config.attackSpeed > 0.0 ? speedError / ctx.config.attackSpeed : 0.0;
    normalizedAccel = std::clamp(normalizedAccel, -1.0, 1.0);
    ctx.decision.accel = static_cast<float>(normalizedAccel);
    return nullptr;
}

const char* StateMoving::name() const {
    return "Moving";
}
