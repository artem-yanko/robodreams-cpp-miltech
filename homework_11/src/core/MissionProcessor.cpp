#include "core/MissionProcessor.hpp"

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

MissionProcessor::MissionProcessor(const RuntimeConfig& config)
    : config(config) {
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
    const double targetDir = std::atan2(dy, dx);
    const double angleError = normalizeAngle(targetDir - static_cast<double>(state.telemetry.dir));
    decision.angleError = angleError;

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

    return decision;
}
