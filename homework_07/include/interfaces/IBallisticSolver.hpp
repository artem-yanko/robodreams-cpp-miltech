#pragma once

#include "domain/types.hpp"

class IBallisticSolver {
public:
    virtual BallisticsResult solve(const Coord& dronePos, const Target& target, double altitude, double attackSpeed, const AmmoParams& ammo) = 0;
    virtual ~IBallisticSolver() {};
};