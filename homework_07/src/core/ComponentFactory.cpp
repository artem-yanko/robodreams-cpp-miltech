#include "core/ComponentFactory.hpp"

#include "providers/AnalyticalSolver.hpp"
#include "providers/JsonConfigLoader.hpp"
#include "providers/JsonTargetProvider.hpp"

IBallisticSolver* ComponentFactory::createSolver(SolverType type) {
    switch (type) {
    case SolverType::ANALYTICAL:
        return new AnalyticalSolver();
    default:
        return nullptr;
    }
}

ITargetProvider* ComponentFactory::createProvider(ProviderType type, const char* param) {
    (void)param;
    switch (type) {
    case ProviderType::JSON:
        return new JsonTargetProvider();
    default:
        return nullptr;
    }
}

IConfigLoader* ComponentFactory::createLoader(LoaderType type) {
    switch (type) {
    case LoaderType::FILE:
        return new JsonConfigLoader();
    default:
        return nullptr;
    }
}

MissionProcessor* ComponentFactory::createMissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader) {
    return new MissionProcessor(targets, solver, configLoader);
}
