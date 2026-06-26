#pragma once

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
    unsigned startLine{24};
    unsigned dropLine{23};
};
