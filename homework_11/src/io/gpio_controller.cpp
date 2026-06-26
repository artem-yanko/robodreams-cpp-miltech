#include "io/gpio_controller.hpp"

bool GpioController::init(const RuntimeConfig& config) {
    mode = config.mode;
    chipName = config.gpioChip;
    startLine = config.startLine;
    dropLine = config.dropLine;
    return true;
}

bool GpioController::setStartHigh() {
    return !chipName.empty();
}

bool GpioController::pulseDrop() {
    return !chipName.empty();
}
