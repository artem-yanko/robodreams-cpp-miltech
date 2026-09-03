#include "autopilot/MissionCompleteState.hpp"

const char* MissionCompleteState::name() const {
    return "MissionCompleteState";
}

bool MissionCompleteState::missionEnabled() const {
    return false;
}

bool MissionCompleteState::dropAllowed() const {
    return false;
}
