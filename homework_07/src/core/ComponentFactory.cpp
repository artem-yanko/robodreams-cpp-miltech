#include "core/ComponentFactory.hpp"

JsonConfigLoader* ComponentFactory::createConfigLoader() {
    return new JsonConfigLoader();
}

JsonTargetProvider* ComponentFactory::createTargetProvider() {
    return new JsonTargetProvider();
}

AnalyticalSolver* ComponentFactory::createSolver() {
    return new AnalyticalSolver();
}

MissionProcessor* ComponentFactory::createMissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader) {
    return new MissionProcessor(targets, solver, configLoader);
}
