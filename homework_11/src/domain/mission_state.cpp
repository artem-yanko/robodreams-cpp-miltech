#include "domain/mission_state.hpp"

void MissionState::updateTarget(const dlink::TargetPos& target) {
    lastTargetUpdate = target;
    targetUpdateReceived = true;

    for (dlink::TargetPos& savedTarget : targets) {
        if (savedTarget.id == target.id) {
            savedTarget = target;
            return;
        }
    }

    targets.push_back(target);
}
