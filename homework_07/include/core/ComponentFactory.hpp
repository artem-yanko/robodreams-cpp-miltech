#pragma once

#include "core/MissionProcessor.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include <memory>

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

class ComponentFactory {
public:
    static std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
    static std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const char* param);
    static std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const char* configPath, const char* ammoPath);
    static std::unique_ptr<MissionProcessor> createMissionProcessor(std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IBallisticSolver> solver, std::unique_ptr<IConfigLoader> configLoader);
};
