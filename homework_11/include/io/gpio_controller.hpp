#pragma once

#include "domain/runtime_config.hpp"

#include <gpiod.h>

#include <string>

class GpioController {
public:
    ~GpioController();

    bool init(const RuntimeConfig& config);
    bool setStartHigh();
    bool pulseDrop();

private:
    void cleanup();
    bool requestLine(unsigned offset, enum gpiod_line_value initialValue, gpiod_line_request** request);
    bool initSimBank(const std::string& bankPath);
    bool writeSimLine(unsigned offset, int value);

    RunMode mode{SIM_MODE};
    std::string chipName{};
    std::string simBank{};
    unsigned startLine{};
    unsigned dropLine{};
    gpiod_chip* chip{nullptr};
    gpiod_line_request* startRequest{nullptr};
    gpiod_line_request* dropRequest{nullptr};
};
