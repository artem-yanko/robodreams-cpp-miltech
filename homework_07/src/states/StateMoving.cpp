#include "states/StateMoving.hpp"

#include "states/StateDecelerating.hpp"
#include "utils/math_utils.hpp"

#include <cmath>

static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;
static const double ANGLE_SPEED_EPSILON = 1e-6;

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.desiredDir);
    double slowTurnThreshold = ctx.activeTurnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    if (std::fabs(angleLeft) > slowTurnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.command.mode = DroneMode::Decelerating;
        ctx.command.angleSpeed = 0.0;
        return std::make_unique<StateDecelerating>();
    }

    ctx.command.mode = DroneMode::Moving;
    if (std::fabs(angleLeft) > ctx.activeTurnThreshold) {
        double turnStep = ctx.config.physicsTimeStep > 0.0 ? ctx.config.physicsTimeStep : ctx.config.simTimeStep;
        double requiredAngleSpeed = turnStep > 0.0 ? angleLeft / turnStep : 0.0;
        double maxAngleSpeed = std::max(0.0, ctx.config.angularSpeed - ANGLE_SPEED_EPSILON);
        if (requiredAngleSpeed > maxAngleSpeed) {
            requiredAngleSpeed = maxAngleSpeed;
        } else if (requiredAngleSpeed < -maxAngleSpeed) {
            requiredAngleSpeed = -maxAngleSpeed;
        }
        ctx.command.angleSpeed = requiredAngleSpeed;
    } else {
        ctx.command.angleSpeed = 0.0;
    }

    return nullptr;
}

const char* StateMoving::name() const {
    return "Moving";
}
