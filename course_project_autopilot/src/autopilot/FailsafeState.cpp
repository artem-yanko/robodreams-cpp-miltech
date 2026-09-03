#include "autopilot/FailsafeState.hpp"

const char* FailsafeState::name() const {
    return "FailsafeState";
}

bool FailsafeState::missionEnabled() const {
    return false;
}

bool FailsafeState::dropAllowed() const {
    return false;
}
