#pragma once
#include "interfaces/IBallisticSolver.hpp"

class AnalyticalSolver : public IBallisticSolver {
public:
    BallisticsResult solve(const Coord& dronePos, const Target& target, double altitude, double attackSpeed, const AmmoParams& ammo) override;
};