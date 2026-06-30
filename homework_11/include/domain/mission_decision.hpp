#pragma once

struct MissionDecision {
    float accel{};
    float turnRate{};
    int targetId{-1};
    double angleError{};
    double distanceToTarget{};
    double predictedTargetX{};
    double predictedTargetY{};
    double dropPointX{};
    double dropPointY{};
    double distanceToDropPoint{};
    double releaseHeading{};
    double releaseTurnThreshold{};
    bool shouldDrop{};
};
