#include <iostream>

#include "core/ComponentFactory.hpp"

int main() {
    JsonConfigLoader* configLoader = ComponentFactory::createConfigLoader();
    JsonTargetProvider* targetProvider = ComponentFactory::createTargetProvider();
    AnalyticalSolver* solver = ComponentFactory::createSolver();

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
