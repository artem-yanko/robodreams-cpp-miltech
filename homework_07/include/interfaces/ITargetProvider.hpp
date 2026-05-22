#pragma once

#include "domain/types.hpp"

class ITargetProvider {
public:
    virtual bool loadTargets(double arrayTimeStep) = 0;
    virtual void setSimulationTime(double time) = 0;
    virtual int getTargetCount() = 0;
    virtual Target getTarget(int index) = 0;
    virtual ~ITargetProvider() {};
};
