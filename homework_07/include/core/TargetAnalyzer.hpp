#pragma once

#include "domain/types.hpp"

class ITargetProvider;

class TargetAnalyzer {
public:
    Coord predictTargetPosition(const Target& target, double flightTime);
    void calculateDropPoint(Coord& dropPoint, Coord& maneuverPoint, bool& needManeuver, const Coord& targetPos, const Coord& dronePos, double horizontalDistance, double remainingAccelerationDistance);
    bool evaluateTarget(BestTargetResult& best, const DroneConfig& config, const DroneMotionState& droneMotion, bool isTurning, bool isMoving, bool isDecelerating, bool isAccelerating, const Target& target, int targetIndex, const Coord& dronePosition, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver);
    bool selectBestTarget(BestTargetResult& result, const DroneConfig& config, const DroneMotionState& droneMotion, bool isTurning, bool isMoving, bool isDecelerating, bool isAccelerating, const Coord& dronePosition, const ITargetProvider& targets, double simulationTime, const BallisticsResult& ballistics, double acceleration, bool returningFromManuver);
};
