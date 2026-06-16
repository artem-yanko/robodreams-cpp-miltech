#include "states/StateAccelerating.hpp"
#include "states/StateMoving.hpp"
#include <cmath>


static double speedLength(const Coord& speed) {
    return std::sqrt(speed.x * speed.x + speed.y * speed.y);
}

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx) {
    ctx.command.mode = DroneMode::Accelerating;
    ctx.command.angleSpeed = 0.0;

    if (speedLength(ctx.telemetry.speed) >= ctx.config.attackSpeed) {
        ctx.command.mode = DroneMode::Moving;
        return std::make_unique<StateMoving>();
    }

    return nullptr;
}

const char* StateAccelerating::name() const {
    return "Accelerating";
}
