#include <iostream>

#include "core/ComponentFactory.hpp"

int main() {
    IConfigLoader* configLoader = ComponentFactory::createLoader(LoaderType::FILE);
    ITargetProvider* targetProvider = ComponentFactory::createProvider(ProviderType::JSON, "targets.json");
    IBallisticSolver* solver = ComponentFactory::createSolver(SolverType::ANALYTICAL);

    if (configLoader == nullptr || targetProvider == nullptr || solver == nullptr) {
        std::cerr << "Failed to create mission components" << std::endl;
        delete solver;
        delete targetProvider;
        delete configLoader;
        return 1;
    }

    MissionProcessor* mission = ComponentFactory::createMissionProcessor(targetProvider, solver, configLoader);
    if (!mission->init()) {
        std::cerr << "Failed to initialize mission" << std::endl;
        delete mission;
        delete solver;
        delete targetProvider;
        delete configLoader;
        return 1;
    }

    while (mission->hasNext()) {
        BallisticsResult result = mission->step();
        std::cout << "flight_time " << result.flightTime << '\n';
        std::cout << "horizontal_distance " << result.horizontalDistance << '\n';
    }

    delete mission;
    delete solver;
    delete targetProvider;
    delete configLoader;

    return 0;
}
