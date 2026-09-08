#pragma once

#include "domain/mission_state.hpp"

#include <cstdint>
#include <string>

class DroneLinkAdapter {
public:
    ~DroneLinkAdapter();

    bool open(const std::string& uartDevice);
    void configureTargetLoss(uint32_t afterMs, uint32_t durationMs);
    int pollIncoming(MissionState& state);
    bool sendControl(float accel, float turnRate);

private:
    bool shouldIgnoreTarget(uint32_t telemetryTimeMs);

    int uartFd{-1};
    dlink::Parser parser{};
    bool debugTargetLossEnabled{};
    bool debugTargetLossActive{};
    uint32_t debugTargetLossAfterMs{};
    uint32_t debugTargetLossDurationMs{};
};
