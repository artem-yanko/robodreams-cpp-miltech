#include "autopilot/AutopilotController.hpp"

#include "autopilot/AutoMissionState.hpp"
#include "autopilot/FailsafeState.hpp"
#include "autopilot/IAutopilotState.hpp"
#include "autopilot/ManualState.hpp"
#include "autopilot/MissionCompleteState.hpp"
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

bool AutopilotController::handleControlLinkLost() {
    if (mode != OperatorMode::Manual) {
        return false;
    }

    setOperatorMode(OperatorMode::Auto);
    return true;
}

bool AutopilotController::handleControlLinkRestored() {
    if (!returnEnabled()) {
        return false;
    }

    setOperatorMode(OperatorMode::Manual);
    return true;
}

bool AutopilotController::enterFailsafeReturn() {
    if (mode != OperatorMode::Auto || returnEnabled()) {
        return false;
    }

    const char* previousStateName = stateName();
    transitionToFailsafe();
    LOG("Autopilot state changed: " << previousStateName << " -> " << stateName());
    return true;
}

bool AutopilotController::completeMission() {
    if (mode != OperatorMode::Auto || !missionEnabled()) {
        return false;
    }

    const char* previousStateName = stateName();
    transitionToMissionComplete();
    LOG("Autopilot state changed: " << previousStateName << " -> " << stateName());
    return true;
}

bool AutopilotController::completeReturn() {
    if (!returnEnabled()) {
        return false;
    }

    const char* previousStateName = stateName();
    transitionToMissionComplete();
    LOG("Autopilot state changed: " << previousStateName << " -> " << stateName());
    return true;
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

bool AutopilotController::returnEnabled() const {
    return state && state->returnEnabled();
}

void AutopilotController::transitionToManual() {
    state = std::make_unique<ManualState>();
}

void AutopilotController::transitionToAutoMission() {
    state = std::make_unique<AutoMissionState>();
}

void AutopilotController::transitionToFailsafe() {
    state = std::make_unique<FailsafeState>();
}

void AutopilotController::transitionToMissionComplete() {
    state = std::make_unique<MissionCompleteState>();
}
