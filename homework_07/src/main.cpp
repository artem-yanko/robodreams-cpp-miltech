#include "utils/logger.hpp"

#include "core/ComponentFactory.hpp"
#include <utility>

int main(int argc, char* argv[]) {
    const char* configPath = argc > 1 ? argv[1] : "config.json";
    const char* ammoPath = argc > 2 ? argv[2] : "ammo.json";
    const char* targetPath = argc > 3 ? argv[3] : "targets.json";
    const char* ballisticTablePath = argc > 4 ? argv[4] : "ballistic_table.txt";

    auto configLoader = ComponentFactory::createLoader(LoaderType::FILE, configPath, ammoPath);
    auto targetProvider = ComponentFactory::createProvider(ProviderType::JSON, targetPath);
    auto solver = ComponentFactory::createSolver(SolverType::TABLE, ballisticTablePath);

    if (configLoader == nullptr || targetProvider == nullptr || solver == nullptr) {
        ERROR_LOG("Failed to create mission components");
        return 1;
    }

    auto mission = ComponentFactory::createMissionProcessor(std::move(targetProvider), std::move(solver), std::move(configLoader));
    if (!mission->init()) {
        ERROR_LOG("Failed to initialize mission");
        return 1;
    }

    while (mission->hasNext()) {
        mission->step();
    }

    return 0;
}
