#pragma once

#include "domain/types.hpp"

class TargetAnalyzer {
public:
    Target analyzeTarget(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep);
    Coord predictTargetPosition(const Target& target, double flightTime);
    void calculateDropPoint(Coord& dropPoint, Coord& maneuverPoint, bool& needManeuver, const Coord& targetPos, const Coord& dronePos, double horizontalDistance, double accelerationPath);
    bool evaluateTarget(BestTargetResult& best, const DroneConfig& config, const DroneMotionState& droneMotion, const TargetData& targets, int targetIndex, const Coord& dronePosition, double simulationTime, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver);
};
