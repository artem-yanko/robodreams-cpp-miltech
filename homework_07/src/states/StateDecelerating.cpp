#include "states/StateDecelerating.hpp"
#include "states/StateTurning.hpp"
#include "states/StateAccelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

static void moveDroneToStop(Coord& position, double direction, double distance) {
  position.x += cos(direction) * distance;
  position.y += sin(direction) * distance;
}

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx) {
  double oldSpeed = ctx.droneMotion.currentSpeed;
  ctx.droneMotion.currentSpeed -= ctx.acceleration * ctx.config.simTimeStep;
  if (ctx.droneMotion.currentSpeed < 0.0) {
    ctx.droneMotion.currentSpeed = 0.0;
  }

  double moveDistance = (oldSpeed + ctx.droneMotion.currentSpeed) * ctx.config.simTimeStep / 2.0;
  moveDroneToStop(ctx.position, ctx.droneMotion.currentDir, moveDistance);
  if (ctx.droneMotion.currentSpeed == 0.0) {
    double turnAngle = std::fabs(calculateAngleDifference(ctx.droneMotion.currentDir, ctx.droneMotion.desiredDir));
    ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
    if (turnAngle > 0.0) {
      ctx.droneMotion.turnRemainingTime = turnAngle / ctx.config.angularSpeed;
      return std::make_unique<StateTurning>();
    } else {
      ctx.droneMotion.turnRemainingTime = 0.0;
      return std::make_unique<StateAccelerating>();
    }
  } else {
    return nullptr;
  }
}

const char* StateDecelerating::name() const {
    return "Decelerating";
}
