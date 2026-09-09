#pragma once

class IAutopilotState {
public:
    virtual ~IAutopilotState() = default;

    virtual const char* name() const = 0;
    virtual bool missionEnabled() const = 0;
    virtual bool dropAllowed() const = 0;
    virtual bool returnEnabled() const { return false; }
};
