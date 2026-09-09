#pragma once

#include "autopilot/OperatorMode.hpp"

#include <memory>

class IAutopilotState;

class AutopilotController {
public:
    AutopilotController();
    ~AutopilotController();

    void setOperatorMode(OperatorMode mode);
    bool handleControlLinkLost();
    bool handleControlLinkRestored();
    bool enterFailsafeReturn();
    bool completeReturn();

    OperatorMode operatorMode() const;
    const char* stateName() const;
    bool missionEnabled() const;
    bool dropAllowed() const;
    bool returnEnabled() const;

private:
    void transitionToManual();
    void transitionToAutoMission();
    void transitionToFailsafe();
    void transitionToMissionComplete();

    OperatorMode mode{OperatorMode::Manual};
    std::unique_ptr<IAutopilotState> state;
};
