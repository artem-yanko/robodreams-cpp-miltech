#include "core/ComponentFactory.hpp"

#include "providers/AnalyticalSolver.hpp"
#include "providers/TableSolver.hpp"

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(
    SolverType type,
    const std::string& ballisticTablePath
) {
    switch (type) {
    case SolverType::TABLE:
        return std::make_unique<TableSolver>(ballisticTablePath);
    case SolverType::ANALYTICAL:
        return std::make_unique<AnalyticalSolver>();
    default:
        return nullptr;
    }
}

std::unique_ptr<JsonConfigLoader> ComponentFactory::createLoader(
    const std::string& configPath,
    const std::string& ammoPath
) {
    return std::make_unique<JsonConfigLoader>(configPath, ammoPath);
}

std::unique_ptr<MissionProcessor> ComponentFactory::createMissionProcessor(
    const RuntimeConfig& config,
    std::unique_ptr<IBallisticSolver> solver
) {
    return std::make_unique<MissionProcessor>(config, std::move(solver));
}

std::unique_ptr<DroneLinkAdapter> ComponentFactory::createDroneLinkAdapter() {
    return std::make_unique<DroneLinkAdapter>();
}

std::unique_ptr<GpioController> ComponentFactory::createGpioController() {
    return std::make_unique<GpioController>();
}
