#pragma once

#include "domain/mission_state.hpp"

#include <cstdint>

struct TargetWaitEvents {
    bool waitingStarted{};
    bool targetAvailable{};
    bool waitTimedOut{};
};

class TargetWaitMonitor {
public:
    TargetWaitEvents update(const MissionState& state, bool enabled);
    bool timeoutReached() const;

private:
    void reset();

    bool waiting{};
    bool timeoutReported{};
    uint32_t waitingSinceMs{};
};
