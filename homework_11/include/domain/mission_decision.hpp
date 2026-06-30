#pragma once

struct MissionDecision {
    float accel{};
    float turnRate{};
    int targetId{-1};
    double angleError{};
    double distanceToTarget{};
    double dropPointX{};
    double dropPointY{};
    double distanceToDropPoint{};
    bool shouldDrop{};
};
