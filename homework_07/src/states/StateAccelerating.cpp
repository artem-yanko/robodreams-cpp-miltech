#include "states/StateAccelerating.hpp"
#include "states/StateMoving.hpp"
#include "utils/math_utils.hpp"

static bool moveDroneToPoint(Coord& position, const Coord& dest, double maxDistance){
  Coord delta = dest - position;
  double distance = length(delta);
  if (distance == 0.0) {
    return true;
  }

  if (maxDistance >= distance) {
      position = dest;
  return true;
  }

  position += normalize(delta) * maxDistance;
  return false;
}

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx) {
  double oldSpeed = ctx.droneMotion.currentSpeed;
  ctx.droneMotion.currentSpeed += ctx.acceleration * ctx.config.simTimeStep;
  if (ctx.droneMotion.currentSpeed > ctx.config.attackSpeed) {
    ctx.droneMotion.currentSpeed = ctx.config.attackSpeed;
  }

  double moveDistance = (oldSpeed + ctx.droneMotion.currentSpeed) * ctx.config.simTimeStep / 2.0;
  ctx.reachedGoal = moveDroneToPoint(ctx.position, ctx.goal, moveDistance);

  if (ctx.droneMotion.currentSpeed >= ctx.config.attackSpeed) {
    return std::make_unique<StateMoving>();
  }
  return nullptr;
}

const char* StateAccelerating::name() const {
    return "Accelerating";
}
