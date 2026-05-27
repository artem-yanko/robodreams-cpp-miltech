#include "utils/logger.hpp"

#include "core/ComponentFactory.hpp"

int main(int argc, char* argv[]) {
    const char* configPath = argc > 1 ? argv[1] : "config.json";
    const char* ammoPath = argc > 2 ? argv[2] : "ammo.json";
    const char* targetPath = argc > 3 ? argv[3] : "targets.json";

    IConfigLoader* configLoader = ComponentFactory::createLoader(LoaderType::FILE, configPath, ammoPath);
    ITargetProvider* targetProvider = ComponentFactory::createProvider(ProviderType::JSON, targetPath);
    IBallisticSolver* solver = ComponentFactory::createSolver(SolverType::ANALYTICAL);

    if (configLoader == nullptr || targetProvider == nullptr || solver == nullptr) {
        ERROR_LOG("Failed to create mission components");
        delete solver;
        delete targetProvider;
        delete configLoader;
        return 1;
    }

    MissionProcessor* mission = ComponentFactory::createMissionProcessor(targetProvider, solver, configLoader);
    if (!mission->init()) {
        ERROR_LOG("Failed to initialize mission");
        delete mission;
        delete solver;
        delete targetProvider;
        delete configLoader;
        return 1;
    }

    while (mission->hasNext()) {
        mission->step();
    }

    delete mission;
    delete solver;
    delete targetProvider;
    delete configLoader;

    return 0;
}
