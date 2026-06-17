#pragma once

#include "domain/types.hpp"

class ITargetProvider {
public:
    virtual ~ITargetProvider() = default;

    virtual bool loadTargets() = 0;
    virtual int getTargetCount() const = 0;
    virtual Target getTarget(int index) const = 0;
    virtual void step(double targetTimeStep, double arrayTimeStep) = 0;
};
