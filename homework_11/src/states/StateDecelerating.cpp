#include "states/StateDecelerating.hpp"

#include "states/StateAccelerating.hpp"
#include "states/StateTurning.hpp"
#include "utils/math_utils.hpp"

#include <cmath>

static const double SPEED_EPSILON = 1e-3;

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx) {
    if (ctx.telemetry.speed <= SPEED_EPSILON) {
        double angleLeft = calculateAngleDifference(ctx.telemetry.dir, ctx.droneMotion.desiredDir);
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        if (std::fabs(angleLeft) > ctx.activeTurnThreshold) {
            ctx.decision.accel = 0.0f;
            ctx.decision.turnRate = angleLeft > 0.0 ? 1.0f : -1.0f;
            return std::make_unique<StateTurning>();
        }

        ctx.decision.accel = 1.0f;
        ctx.decision.turnRate = 0.0f;
        return std::make_unique<StateAccelerating>();
    }

    ctx.decision.accel = -1.0f;
    ctx.decision.turnRate = 0.0f;
    return nullptr;
}

const char* StateDecelerating::name() const {
    return "Decelerating";
}
