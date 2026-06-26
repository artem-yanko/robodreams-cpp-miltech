#include "domain/mission_state.hpp"
#include "domain/runtime_config.hpp"
#include "io/drone_link_adapter.hpp"
#include "io/gpio_controller.hpp"
#include "utils/logger.hpp"

#include <cstring>
#include <string>

static RuntimeConfig parseArgs(int argc, char* argv[]) {
    RuntimeConfig config{};

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--sim") == 0) {
            config.mode = SIM_MODE;
            config.uartDevice = "/tmp/ttyA";
            config.gpioChip = "gpiochip1";
            config.startLine = 24;
            config.dropLine = 23;
        } else if (std::strcmp(argv[i], "--hw") == 0) {
            config.mode = HW_MODE;
            config.uartDevice = "/dev/ttyAMA1";
            config.gpioChip = "gpiochip0";
            config.startLine = 27;
            config.dropLine = 22;
        } else if (std::strcmp(argv[i], "--uart") == 0 && i + 1 < argc) {
            config.uartDevice = argv[++i];
        } else if (std::strcmp(argv[i], "--gpiochip") == 0 && i + 1 < argc) {
            config.gpioChip = argv[++i];
        } else if (std::strcmp(argv[i], "--start-line") == 0 && i + 1 < argc) {
            config.startLine = static_cast<unsigned>(std::stoul(argv[++i]));
        } else if (std::strcmp(argv[i], "--drop-line") == 0 && i + 1 < argc) {
            config.dropLine = static_cast<unsigned>(std::stoul(argv[++i]));
        }
    }

    return config;
}

int main(int argc, char* argv[]) {
    RuntimeConfig config = parseArgs(argc, argv);
    MissionState state{};

    DroneLinkAdapter link;
    if (!link.open(config.uartDevice)) {
        ERROR_LOG("Failed to open UART link");
        return 1;
    }

    GpioController gpio;
    if (!gpio.init(config)) {
        ERROR_LOG("Failed to initialize GPIO");
        return 1;
    }

    if (!gpio.setStartHigh()) {
        ERROR_LOG("Failed to raise START line");
        return 1;
    }
    state.startRaised = true;

    LOG("Homework 11 skeleton initialized");
    LOG("Mode=" << (config.mode == SIM_MODE ? "sim" : "hw")
        << ", uart=" << config.uartDevice
        << ", gpio=" << config.gpioChip
        << ", startLine=" << config.startLine
        << ", dropLine=" << config.dropLine);

    return 0;
}
