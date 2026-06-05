#pragma once

#include "domain/types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include <memory>
#include <unordered_map>
#include <vector>

class MissionProcessor {
public:
    ~MissionProcessor();
    MissionProcessor(std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IBallisticSolver> solver, std::unique_ptr<IConfigLoader> configLoader);

    bool init();
    bool hasNext();
    BallisticsResult step();
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> newSolver);

private:
    std::unique_ptr<ITargetProvider> targets;
    std::unique_ptr<IBallisticSolver> solver;
    std::unique_ptr<IConfigLoader> configLoader;

    DroneConfig config{};
    std::vector<AmmoParams> ammoList{};
    std::unordered_map<std::string, std::size_t> ammoIndex{};
    BallisticsResult ballistics{};
    double simulationTime{};
    Coord dronePosition{};
    DroneMotionState droneMotion{};
    double acceleration{};
    bool missionComplete{};
    bool targetLocked{};
    int lockedTargetIndex{};
    bool returningFromManuver{};
    int maneuverTargetIndex{};
    std::vector<SimStep> steps{};
};
