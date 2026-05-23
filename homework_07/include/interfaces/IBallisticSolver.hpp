#pragma once

#include "domain/types.hpp"

class IBallisticSolver {
public:
    virtual BallisticsResult solve(const DroneConfig& config, const AmmoParams& ammo) = 0;
    virtual ~IBallisticSolver() {};
};
