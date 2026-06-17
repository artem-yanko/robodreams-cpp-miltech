#include "utils/logger.hpp"

#include "core/ComponentFactory.hpp"
#include <chrono>
#include <thread>
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

    ITargetProvider& targetProviderRef = mission->getTargetProvider();
    DronePhysics& dronePhysicsRef = mission->getDronePhysics();
    const DroneConfig& config = mission->getConfig();

    std::thread providerThread([&]() {
        targetProviderRef.run(config.targetTimeStep, config.arrayTimeStep, config.timeScale);
    });
    std::thread physicsThread([&]() {
        dronePhysicsRef.run();
    });
    std::thread missionThread([&]() {
        mission->run();
    });

    while (!targetProviderRef.isThreadReady() || !dronePhysicsRef.isThreadReady() || !mission->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    targetProviderRef.start();
    dronePhysicsRef.start();
    mission->start();

    missionThread.join();
    targetProviderRef.stop();
    dronePhysicsRef.stop();
    providerThread.join();
    physicsThread.join();

    return 0;
}
