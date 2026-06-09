#include "states/StateStopped.hpp"
#include "states/StateTurning.hpp"
#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx) {
    ctx.droneMotion.currentSpeed = 0.0;

    double deltaAngle = std::fabs(calculateAngleDifference(ctx.droneMotion.currentDir, ctx.droneMotion.desiredDir));
    if (deltaAngle > ctx.config.turnThreshold) {
        ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
        ctx.droneMotion.turnRemainingTime = deltaAngle / ctx.config.angularSpeed;
        return std::make_unique<StateTurning>();
    } else {
        ctx.droneMotion.currentDir = ctx.droneMotion.desiredDir;
        return std::make_unique<StateAccelerating>();
    }
}

const char* StateStopped::name() const {
    return "Stopped";
}

