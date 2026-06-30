#pragma once

#include "domain/types.hpp"
#include "utils/drone_link.hpp"

#include <vector>

struct TargetTrack {
    uint8_t id{};
    Coord velocity{};
    uint32_t updatedAtMs{};
    bool hasVelocity{};
};

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
    std::vector<TargetTrack> targetTracks{};

    void updateTarget(const dlink::TargetPos& target, uint32_t timeMs);
};
