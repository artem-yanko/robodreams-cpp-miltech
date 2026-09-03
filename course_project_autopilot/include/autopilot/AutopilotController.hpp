#pragma once

#include "autopilot/OperatorMode.hpp"

#include <memory>

class IAutopilotState;

class AutopilotController {
public:
    AutopilotController();
    ~AutopilotController();

    void setOperatorMode(OperatorMode mode);

    OperatorMode operatorMode() const;
    const char* stateName() const;
    bool missionEnabled() const;
    bool dropAllowed() const;

private:
    void transitionToManual();
    void transitionToAutoMission();

    OperatorMode mode{OperatorMode::Manual};
    std::unique_ptr<IAutopilotState> state;
};
