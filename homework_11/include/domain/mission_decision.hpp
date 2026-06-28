#pragma once

struct MissionDecision {
    float accel{};
    float turnRate{};
    int targetId{-1};
    double angleError{};
};
