#pragma once

#include "domain/types.hpp"
#include "utils/drone_link.hpp"

#include <cstddef>
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
    bool droneCfgReceived{};
    bool targetUpdateReceived{};
    dlink::Telemetry telemetry{};
    dlink::AmmoCfg ammo{};
    dlink::DroneCfg droneCfg{};
    dlink::TargetPos lastTargetUpdate{};
    std::vector<dlink::TargetPos> targets{};
    std::vector<TargetTrack> targetTracks{};

    void updateTarget(const dlink::TargetPos& target, uint32_t timeMs);
    bool isTargetActive(std::size_t targetIndex) const;
    bool hasActiveTargets() const;
};
