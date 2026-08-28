#pragma once

#include "domain/types.hpp"
#include <string>

enum RunMode {
    SIM_MODE,
    HW_MODE
};

struct RuntimeConfig {
    RunMode mode{SIM_MODE};
    std::string uartDevice{"/tmp/ttyA"};
    std::string gpioChip{"gpiochip1"};
    std::string simBank{};
    std::string configPath{};
    std::string ammoPath{};
    std::string ballisticTablePath{};
    std::string mavlinkHost{"127.0.0.1"};
    unsigned startLine{24};
    unsigned dropLine{23};
    unsigned mavlinkPort{14550};
    bool mavlinkEnabled{};
    DroneConfig drone{};
};
