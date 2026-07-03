#pragma once

#include "interfaces/IDroneState.hpp"

class StateTurning : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override;
    bool isTurning() const override { return true; }
};
