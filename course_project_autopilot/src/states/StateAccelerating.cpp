#include "states/StateAccelerating.hpp"

#include "states/StateDecelerating.hpp"
#include "states/StateMoving.hpp"
#include "utils/math_utils.hpp"

#include <cmath>

static const double SPEED_EPSILON = 1e-6;
static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.dir, ctx.droneMotion.desiredDir);
    double slowTurnThreshold = ctx.activeTurnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
    if (std::fabs(angleLeft) > slowTurnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.decision.accel = -1.0f;
        ctx.decision.turnRate = 0.0f;
        return std::make_unique<StateDecelerating>();
    }

    if (ctx.telemetry.speed >= ctx.config.attackSpeed - SPEED_EPSILON) {
        ctx.decision.accel = 0.0f;
        ctx.decision.turnRate = 0.0f;
        return std::make_unique<StateMoving>();
    }

    ctx.decision.accel = 1.0f;
    ctx.decision.turnRate = 0.0f;
    return nullptr;
}

const char* StateAccelerating::name() const {
    return "Accelerating";
}
