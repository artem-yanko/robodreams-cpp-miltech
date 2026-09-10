#pragma once

#include "autopilot/IAutopilotState.hpp"

class MissionCompleteState : public IAutopilotState {
public:
    const char* name() const override;
    bool missionEnabled() const override;
    bool dropAllowed() const override;
};
