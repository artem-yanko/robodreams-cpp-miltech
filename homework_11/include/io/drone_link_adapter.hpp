#pragma once

#include "domain/mission_state.hpp"

#include <string>

class DroneLinkAdapter {
public:
    ~DroneLinkAdapter();

    bool open(const std::string& uartDevice);
    int pollIncoming(MissionState& state);
    bool sendControl(float accel, float turnRate);

private:
    int uartFd{-1};
    dlink::Parser parser{};
};
