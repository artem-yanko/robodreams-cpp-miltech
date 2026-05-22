#pragma once

#include "domain/types.hpp"

class ITargetProvider {
public:
    virtual bool loadTargets() = 0;
    virtual int getTargetCount() = 0;
    virtual const TargetData& getTargetsData() = 0;
    virtual ~ITargetProvider() {};
};
