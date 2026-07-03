#include "core/ComponentFactory.hpp"

#include "providers/AnalyticalSolver.hpp"
#include "providers/JsonConfigLoader.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "providers/TableSolver.hpp"
std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type, const char* param) {
    switch (type) {
    case SolverType::ANALYTICAL:
        return std::make_unique<AnalyticalSolver>();
    case SolverType::TABLE:
        return std::make_unique<TableSolver>(param != nullptr ? param : "ballistic_table.txt");
    default:
        return nullptr;
    }
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType type, const char* param) {
    (void)param;
    switch (type) {
    case ProviderType::JSON:
        return std::make_unique<JsonTargetProvider>(param != nullptr ? param : "targets.json");
    default:
        return nullptr;
    }
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(LoaderType type, const char* configPath, const char* ammoPath) {
    switch (type) {
    case LoaderType::FILE:
        return std::make_unique<JsonConfigLoader>(configPath != nullptr ? configPath : "config.json", ammoPath != nullptr ? ammoPath : "ammo.json");
    default:
        return nullptr;
    }
}

std::unique_ptr<MissionProcessor> ComponentFactory::createMissionProcessor(std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IBallisticSolver> solver, std::unique_ptr<IConfigLoader> configLoader) {
    return std::make_unique<MissionProcessor>(std::move(targets), std::move(solver), std::move(configLoader));
}
