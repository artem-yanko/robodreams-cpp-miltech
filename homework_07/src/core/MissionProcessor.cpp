#include "core/MissionProcessor.hpp"
#include "core/TargetAnalyzer.hpp"
#include "utils/logger.hpp"
#include <cstring>
#include <cmath>

MissionProcessor::~MissionProcessor() {
    if (ammoList) {
        delete[] ammoList;
        ammoList = nullptr;
    }
}

MissionProcessor::MissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* configLoader) : targets(targets), solver(solver), configLoader(configLoader) {
        ammoList = nullptr;
        ammoCount = 0;
        currentIndex = 0;
        simulationTime = 0.0;
    }

bool MissionProcessor::init() {
    currentIndex = 0;
    simulationTime = 0.0;
    if (!configLoader->loadConfig(config)) {
        ERROR_LOG("Failed to load drone configuration");
        return false;
    }
    LOG("Configuration loaded: attackSpeed=" << config.attackSpeed
        << ", accelPath=" << config.accelPath
        << ", ammoName=" << config.ammoName
        << ", startPos=(" << config.startPos.x << "," << config.startPos.y << ")");

    if (!configLoader->loadAmmo(ammoList, ammoCount)) {
        ERROR_LOG("Failed to load ammo parameters");
        return false;
    }
    LOG("Ammo parameters loaded: " << ammoCount);

    if (!targets->loadTargets()) {
        ERROR_LOG("Failed to load targets");
        return false;
    }

    return true;
}

bool MissionProcessor::hasNext() {
    return currentIndex < targets->getTargetCount();
}

static const AmmoParams* ammoSelect(const AmmoParams* ammoList, int ammoCount, const char* ammoName) {
    for (int i = 0; i < ammoCount; ++i) {
        if (strcmp(ammoName, ammoList[i].name) == 0) {
            return &ammoList[i];
        }
    }
    return nullptr;
}

BallisticsResult MissionProcessor::step() {
    TargetAnalyzer analyzer;
    Target target = analyzer.analyzeTarget(currentIndex, targets->getTargetsData(), simulationTime, config.arrayTimeStep);
    
    const AmmoParams* selectedAmmo = ammoSelect(ammoList, ammoCount, config.ammoName);
    if (selectedAmmo == nullptr) {
        ERROR_LOG("Failed to select ammo parameters");
        return {};
    }

    BallisticsResult result = solver->solve(config.startPos, target, config.altitude, config.attackSpeed, *selectedAmmo);
    currentIndex++;
    simulationTime += config.simTimeStep;
    return result;
}

void MissionProcessor::reset() {
    currentIndex = 0;
    simulationTime = 0.0;
}

void MissionProcessor::changeSolver(IBallisticSolver* newSolver) {
    solver = newSolver;
}
