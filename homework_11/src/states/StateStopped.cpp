#include "states/StateStopped.hpp"

#include "states/StateAccelerating.hpp"
#include "states/StateTurning.hpp"
#include "utils/math_utils.hpp"

#include <cmath>

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx) {
    double angleLeft = calculateAngleDifference(ctx.telemetry.dir, ctx.droneMotion.desiredDir);
    if (std::fabs(angleLeft) > ctx.activeTurnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.decision.accel = 0.0f;
        ctx.decision.turnRate = angleLeft > 0.0 ? 1.0f : -1.0f;
        return std::make_unique<StateTurning>();
    }

    ctx.decision.accel = 1.0f;
    ctx.decision.turnRate = 0.0f;
    return std::make_unique<StateAccelerating>();
}

const char* StateStopped::name() const {
    return "Stopped";
}
