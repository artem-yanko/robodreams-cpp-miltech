#include "states/StateMoving.hpp"
#include "states/StateDecelerating.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

static const double SLOW_TURN_THRESHOLD_FACTOR = 3.0;

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

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx) {
  double angleLeft = calculateAngleDifference(ctx.droneMotion.currentDir, ctx.droneMotion.desiredDir);
  double turnStep = ctx.config.angularSpeed * ctx.config.simTimeStep;
  double slowTurnThreshold = ctx.config.turnThreshold * SLOW_TURN_THRESHOLD_FACTOR;
  bool shouldStopForTurn = std::fabs(angleLeft) > slowTurnThreshold;
  if (shouldStopForTurn) {
      ctx.droneMotion.turnTargetDir = ctx.droneMotion.desiredDir;
      if (ctx.droneMotion.currentSpeed > 0.0) {
        return std::make_unique<StateDecelerating>();
      }
    }
  if (std::fabs(angleLeft) <= slowTurnThreshold) {
    if (std::fabs(angleLeft) <= turnStep) {
      ctx.droneMotion.currentDir = ctx.droneMotion.desiredDir;
    } else if (angleLeft > 0.0) {
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
  }

  ctx.droneMotion.currentSpeed = ctx.config.attackSpeed;
  ctx.reachedGoal = moveDroneToPoint(ctx.position, ctx.goal, ctx.droneMotion.currentSpeed * ctx.config.simTimeStep);
  return nullptr;
}

const char* StateMoving::name() const {
    return "Moving";
}

