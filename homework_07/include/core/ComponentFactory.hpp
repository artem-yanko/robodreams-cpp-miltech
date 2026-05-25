#pragma once

#include "core/MissionProcessor.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

class ComponentFactory {
public:
    static IBallisticSolver* createSolver(SolverType type);
    static ITargetProvider* createProvider(ProviderType type, const char* param);
    static IConfigLoader* createLoader(LoaderType type, const char* configPath, const char* ammoPath);
    static MissionProcessor* createMissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader);
};
