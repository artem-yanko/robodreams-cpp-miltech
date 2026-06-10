#include "states/StateTurning.hpp"
#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx) {
    ctx.droneMotion.phase = TURNING;
    double angleLeft = calculateAngleDifference(ctx.droneMotion.currentDir, ctx.droneMotion.turnTargetDir);
    double turnStep = ctx.config.angularSpeed * ctx.config.simTimeStep;

    if (std::fabs(angleLeft) <= turnStep) {
      ctx.droneMotion.currentDir = ctx.droneMotion.turnTargetDir;
      ctx.droneMotion.turnRemainingTime = 0.0;
      return std::make_unique<StateAccelerating>();
    } else {
      if (angleLeft > 0.0) {
        ctx.droneMotion.currentDir += turnStep;
        if (ctx.droneMotion.currentDir > M_PI) {
          ctx.droneMotion.currentDir -= 2.0 * M_PI;
        }
      } else {
        ctx.droneMotion.currentDir -= turnStep;
        if (ctx.droneMotion.currentDir < -M_PI) {
          ctx.droneMotion.currentDir += 2.0 * M_PI;
        }
      }
      ctx.droneMotion.turnRemainingTime = (std::fabs(angleLeft) - turnStep) / ctx.config.angularSpeed;
      return nullptr;
    }
}

const char* StateTurning::name() const {
    return "Turning";
}

