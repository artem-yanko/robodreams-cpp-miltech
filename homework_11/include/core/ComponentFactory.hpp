#pragma once

#include "core/MissionProcessor.hpp"
#include "io/drone_link_adapter.hpp"
#include "io/gpio_controller.hpp"
#include "providers/JsonConfigLoader.hpp"

#include <memory>
#include <string>

class IBallisticSolver;

enum class SolverType {
    ANALYTICAL,
    TABLE
};

class ComponentFactory {
public:
    static std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const std::string& ballisticTablePath);
    static std::unique_ptr<JsonConfigLoader> createLoader(const std::string& configPath, const std::string& ammoPath);
    static std::unique_ptr<MissionProcessor> createMissionProcessor(
        const RuntimeConfig& config,
        std::unique_ptr<IBallisticSolver> solver);
    static std::unique_ptr<DroneLinkAdapter> createDroneLinkAdapter();
    static std::unique_ptr<GpioController> createGpioController();
};
