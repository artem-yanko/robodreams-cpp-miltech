#pragma once

#include "utils/drone_link.hpp"

#include <vector>

struct MissionState {
    bool startRaised{};
    bool dropDone{};
    bool telemetryReceived{};
    bool ammoReceived{};
    bool targetUpdateReceived{};
    dlink::Telemetry telemetry{};
    dlink::AmmoCfg ammo{};
    dlink::TargetPos lastTargetUpdate{};
    std::vector<dlink::TargetPos> targets{};

    void updateTarget(const dlink::TargetPos& target);
};
