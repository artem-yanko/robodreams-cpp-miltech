#include "autopilot/ManualState.hpp"

const char* ManualState::name() const {
    return "ManualState";
}

bool ManualState::missionEnabled() const {
    return false;
}

bool ManualState::dropAllowed() const {
    return false;
}
