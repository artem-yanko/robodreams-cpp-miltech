#include "autopilot/TargetWaitMonitor.hpp"

namespace {

constexpr uint32_t WAIT_FOR_TARGETS_TIMEOUT_MS = 30'000;

}

TargetWaitEvents TargetWaitMonitor::update(const MissionState& state, bool enabled) {
    TargetWaitEvents events{};
    if (!enabled || !state.telemetryReceived) {
        reset();
        return events;
    }

    if (state.hasActiveTargets()) {
        if (waiting) {
            events.targetAvailable = true;
        }
        reset();
        return events;
    }

    if (!waiting) {
        waiting = true;
        waitingSinceMs = state.telemetry.t_ms;
        events.waitingStarted = true;
        return events;
    }

    if (!timeoutReported
        && state.telemetry.t_ms - waitingSinceMs > WAIT_FOR_TARGETS_TIMEOUT_MS) {
        timeoutReported = true;
        events.waitTimedOut = true;
    }

    return events;
}

bool TargetWaitMonitor::timeoutReached() const {
    return timeoutReported;
}

void TargetWaitMonitor::reset() {
    waiting = false;
    timeoutReported = false;
    waitingSinceMs = 0;
}
