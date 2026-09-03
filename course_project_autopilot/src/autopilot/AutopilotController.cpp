#include "autopilot/AutopilotController.hpp"

#include "autopilot/AutoMissionState.hpp"
#include "autopilot/IAutopilotState.hpp"
#include "autopilot/ManualState.hpp"
#include "utils/logger.hpp"

AutopilotController::AutopilotController()
    : state(std::make_unique<ManualState>()) {
}

AutopilotController::~AutopilotController() = default;

void AutopilotController::setOperatorMode(OperatorMode nextMode) {
    if (nextMode == mode) {
        return;
    }

    const char* previousModeName = operatorModeName(mode);
    const char* previousStateName = stateName();
    mode = nextMode;

    switch (mode) {
    case OperatorMode::Manual:
        transitionToManual();
        break;
    case OperatorMode::Auto:
        transitionToAutoMission();
        break;
    }

    LOG("Autopilot mode changed: " << previousModeName
        << " -> " << operatorModeName(mode)
        << ", state=" << previousStateName
        << " -> " << stateName());
}

OperatorMode AutopilotController::operatorMode() const {
    return mode;
}

const char* AutopilotController::stateName() const {
    return state ? state->name() : "UnknownState";
}

bool AutopilotController::missionEnabled() const {
    return state && state->missionEnabled();
}

bool AutopilotController::dropAllowed() const {
    return state && state->dropAllowed();
}

void AutopilotController::transitionToManual() {
    state = std::make_unique<ManualState>();
}

void AutopilotController::transitionToAutoMission() {
    state = std::make_unique<AutoMissionState>();
}
