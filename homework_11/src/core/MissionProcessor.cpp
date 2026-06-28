#include "core/MissionProcessor.hpp"
#include "interfaces/IBallisticSolver.hpp"

#include <algorithm>
#include <cmath>

static double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle;
}

MissionProcessor::MissionProcessor(const RuntimeConfig& config, std::unique_ptr<IBallisticSolver> solver)
    : config(config), solver(std::move(solver)) {
}

MissionDecision MissionProcessor::update(const MissionState& state) const {
    MissionDecision decision{};

    if (!state.telemetryReceived || state.targets.empty()) {
        return decision;
    }

    const dlink::TargetPos& target = state.targets.front();
    decision.targetId = static_cast<int>(target.id);
    const double dx = static_cast<double>(target.x) - static_cast<double>(state.telemetry.x);
    const double dy = static_cast<double>(target.y) - static_cast<double>(state.telemetry.y);
    const double distanceToTarget = std::hypot(dx, dy);
    const double targetDir = std::atan2(dy, dx);
    const double angleError = normalizeAngle(targetDir - static_cast<double>(state.telemetry.dir));
    decision.angleError = angleError;
    decision.distanceToTarget = distanceToTarget;

    AmmoParams ammo{};
    if (state.ammoReceived) {
        ammo.name = state.ammo.name;
        ammo.mass = state.ammo.mass;
        ammo.drag = state.ammo.drag;
        ammo.lift = state.ammo.lift;
    } else {
        ammo.name = config.drone.ammoName;
    }

    BallisticsResult ballistics{};
    if (solver != nullptr && state.ammoReceived) {
        ballistics = solver->solve(config.drone, ammo);
    }

    const double normalizedTurn = angleError / config.drone.angularSpeed;
    decision.turnRate = static_cast<float>(std::clamp(normalizedTurn, -1.0, 1.0));

    const double absAngleError = std::abs(angleError);
    if (absAngleError <= config.drone.turnThreshold) {
        decision.accel = 1.0f;
    } else if (absAngleError <= config.drone.turnThreshold * 3.0) {
        decision.accel = 0.3f;
    } else {
        decision.accel = 0.0f;
    }

    if (!state.dropDone
        && ballistics.horizontalDistance > 0.0
        && distanceToTarget <= ballistics.horizontalDistance
        && absAngleError <= config.drone.turnThreshold) {
        decision.shouldDrop = true;
    }

    return decision;
}
