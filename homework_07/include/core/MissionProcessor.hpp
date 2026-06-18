#pragma once

#include "core/DronePhysics.hpp"
#include "domain/types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IDroneState.hpp"
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>

class MissionProcessor {
public:
    ~MissionProcessor();
    MissionProcessor(std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IBallisticSolver> solver, std::unique_ptr<IConfigLoader> configLoader);

    void run();
    void start();
    void stop();
    bool isThreadReady() const;

    bool init();
    bool hasNext();
    BallisticsResult step();
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> newSolver);
    ITargetProvider& getTargetProvider();
    DronePhysics& getDronePhysics();
    const DroneConfig& getConfig() const;

private:
    std::unique_ptr<ITargetProvider> targets;
    std::unique_ptr<IBallisticSolver> solver;
    std::unique_ptr<IConfigLoader> configLoader;
    std::unique_ptr<IDroneState> droneState;
    std::unique_ptr<DronePhysics> dronePhysics;

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
    int candidateTargetIndex{};
    int candidateTargetStreak{};
    bool returningFromManuver{};
    int maneuverTargetIndex{};
    bool hasPreviousLockedPlan_{false};
    int previousLockedPlanTargetIndex_{-1};
    bool previousLockedPlanNeedManeuver_{false};
    Coord previousLockedPlanDropPoint_{};
    Coord previousLockedPlanPredictedTarget_{};
    std::vector<SimStep> steps{};
    std::atomic<bool> running_{true};
    std::atomic<bool> started_{false};
    std::atomic<bool> threadReady_{false};
};
