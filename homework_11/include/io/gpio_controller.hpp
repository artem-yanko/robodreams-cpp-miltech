#pragma once

#include "domain/runtime_config.hpp"
#include <string>

class GpioController {
public:
    bool init(const RuntimeConfig& config);
    bool setStartHigh();
    bool pulseDrop();

private:
    RunMode mode{SIM_MODE};
    std::string chipName{};
    unsigned startLine{};
    unsigned dropLine{};
};
