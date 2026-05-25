#include <iostream>

#include "core/ComponentFactory.hpp"

int main(int argc, char* argv[]) {
    const char* configPath = argc > 1 ? argv[1] : "config.json";
    const char* ammoPath = argc > 2 ? argv[2] : "ammo.json";
    const char* targetPath = argc > 3 ? argv[3] : "targets.json";

    IConfigLoader* configLoader = ComponentFactory::createLoader(LoaderType::FILE, configPath, ammoPath);
    ITargetProvider* targetProvider = ComponentFactory::createProvider(ProviderType::JSON, targetPath);
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
