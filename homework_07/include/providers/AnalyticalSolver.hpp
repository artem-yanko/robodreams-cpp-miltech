#pragma once
#include "interfaces/IBallisticSolver.hpp"

class AnalyticalSolver : public IBallisticSolver {
public:
    BallisticsResult solve(const DroneConfig& config, const AmmoParams& ammo) override;
};
