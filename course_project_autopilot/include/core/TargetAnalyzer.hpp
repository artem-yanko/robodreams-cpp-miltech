#pragma once

#include "domain/mission_state.hpp"
#include "domain/types.hpp"

class TargetAnalyzer {
public:
    Target analyzeTarget(const MissionState& state, std::size_t targetIndex) const;
    Coord predictTargetPosition(const Target& target, double flightTime) const;
    void calculateDropPoint(
        Coord& dropPoint,
        Coord& maneuverPoint,
        bool& needManeuver,
        const Coord& targetPos,
        const Coord& dronePos,
        double horizontalDistance,
        double accelerationPath
    ) const;
    bool evaluateTarget(
        BestTargetResult& best,
        const MissionState& state,
        const DroneConfig& config,
        const DroneMotionState& droneMotion,
        bool isTurning,
        bool isMoving,
        bool isDecelerating,
        bool isAccelerating,
        std::size_t targetIndex,
        const Coord& dronePosition,
        const BallisticsResult& ballistics,
        double acceleration,
        bool returningFromManuver
    ) const;
    bool selectBestTarget(
        BestTargetResult& result,
        const MissionState& state,
        const DroneConfig& config,
        const DroneMotionState& droneMotion,
        bool isTurning,
        bool isMoving,
        bool isDecelerating,
        bool isAccelerating,
        const Coord& dronePosition,
        const BallisticsResult& ballistics,
        double acceleration,
        bool returningFromManuver
    ) const;
};
