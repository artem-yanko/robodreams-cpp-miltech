#include <iostream>

#include "providers/AnalyticalSolver.hpp"
#include "providers/JsonConfigLoader.hpp"

int main() {
    AnalyticalSolver solver;
    JsonConfigLoader jsonConfigLoader;

    Coord dronePos{0.0, 0.0};
    Target target{};
    target.position = {100.0, 50.0};
    target.velocity = {0.0, 0.0};

    AmmoParams ammo{};
    ammo.mass = 0.35;
    ammo.drag = 0.08;
    ammo.lift = 0.15;

    const double altitude = 120.0;
    const double attackSpeed = 22.0;

    BallisticsResult result = solver.solve(dronePos, target, altitude, attackSpeed, ammo);
    DroneConfig config;
    if (!jsonConfigLoader.loadConfig(config)) {
        std::cerr << "Failed to load config" << std::endl;
        return 1;
    }

    std::cout << "flight_time " << result.flightTime << '\n';
    std::cout << "horizontal_distance " << result.horizontalDistance << '\n';
    std::cout << "config attackSpeed " << config.attackSpeed << '\n';

    return 0;
}
