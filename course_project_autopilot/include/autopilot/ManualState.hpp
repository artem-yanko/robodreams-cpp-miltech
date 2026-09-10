#pragma once

#include "autopilot/IAutopilotState.hpp"

class ManualState : public IAutopilotState {
public:
    const char* name() const override;
    bool missionEnabled() const override;
    bool dropAllowed() const override;
};
