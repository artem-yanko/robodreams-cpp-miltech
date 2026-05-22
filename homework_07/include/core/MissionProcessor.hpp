#pragma once

#include "domain/types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"

class MissionProcessor {
public:
    ~MissionProcessor();
    MissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader);

    bool init();
    bool hasNext();
    BallisticsResult step();
    void reset();
    void changeSolver(IBallisticSolver* newSolver);

private:
    ITargetProvider* targets;
    IBallisticSolver* solver;
    IConfigLoader* configLoader;

    DroneConfig config{};
    AmmoParams* ammoList{};
    int ammoCount{};
    int currentIndex{};
};