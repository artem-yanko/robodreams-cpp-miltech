#include "states/StateAccelerating.hpp"
#include "states/StateDecelerating.hpp"
#include "states/StateMoving.hpp"
#include "utils/math_utils.hpp"

#include <cmath>


static double speedLength(const Coord& speed) {
    return std::sqrt(speed.x * speed.x + speed.y * speed.y);
}

static constexpr double SPEED_EPSILON = 1e-6;
static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.direction, ctx.droneMotion.desiredDir);
    double slowTurnThreshold = ctx.config.turnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    if (std::fabs(angleLeft) > slowTurnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.command.mode = DroneMode::Decelerating;
        ctx.command.angleSpeed = 0.0;
        return std::make_unique<StateDecelerating>();
    }

    ctx.command.mode = DroneMode::Accelerating;
    ctx.command.angleSpeed = 0.0;

    if (speedLength(ctx.telemetry.speed) >= ctx.config.attackSpeed - SPEED_EPSILON) {
        ctx.command.mode = DroneMode::Moving;
        return std::make_unique<StateMoving>();
    }

    return nullptr;
}

const char* StateAccelerating::name() const {
    return "Accelerating";
}
