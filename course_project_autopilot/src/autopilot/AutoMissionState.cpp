#include "autopilot/AutoMissionState.hpp"

const char* AutoMissionState::name() const {
    return "AutoMissionState";
}

bool AutoMissionState::missionEnabled() const {
    return true;
}

bool AutoMissionState::dropAllowed() const {
    return true;
}
