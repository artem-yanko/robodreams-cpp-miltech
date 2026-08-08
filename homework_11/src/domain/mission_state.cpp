#include "domain/mission_state.hpp"

#include <cstddef>

void MissionState::updateTarget(const dlink::TargetPos& target, uint32_t timeMs) {
    lastTargetUpdate = target;
    targetUpdateReceived = true;

    std::size_t savedIndex = targets.size();
    dlink::TargetPos previousTarget{};
    for (dlink::TargetPos& savedTarget : targets) {
        if (savedTarget.id == target.id) {
            savedIndex = static_cast<std::size_t>(&savedTarget - targets.data());
            previousTarget = savedTarget;
            savedTarget = target;
            break;
        }
    }

    if (savedIndex == targets.size()) {
        targets.push_back(target);
        targetTracks.push_back(TargetTrack{target.id, {}, timeMs, false});
        return;
    }

    if (savedIndex >= targetTracks.size()) {
        targetTracks.resize(targets.size());
    }

    TargetTrack& track = targetTracks[savedIndex];
    if (track.id != target.id) {
        track.id = target.id;
        track.velocity = {};
        track.updatedAtMs = timeMs;
        track.hasVelocity = false;
        return;
    }

    if (timeMs > track.updatedAtMs) {
        const double dt = static_cast<double>(timeMs - track.updatedAtMs) / 1000.0;
        if (dt > 0.0) {
            track.velocity = Coord{
                (target.x - previousTarget.x) / dt,
                (target.y - previousTarget.y) / dt
            };
            track.hasVelocity = true;
        }
    }

    track.updatedAtMs = timeMs;
}
