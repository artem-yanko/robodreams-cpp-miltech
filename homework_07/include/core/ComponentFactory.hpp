#pragma once

#include "core/MissionProcessor.hpp"
#include "providers/AnalyticalSolver.hpp"
#include "providers/JsonConfigLoader.hpp"
#include "providers/JsonTargetProvider.hpp"

class ComponentFactory {
public:
    static JsonConfigLoader* createConfigLoader();
    static JsonTargetProvider* createTargetProvider();
    static AnalyticalSolver* createSolver();
    static MissionProcessor* createMissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader);
};
