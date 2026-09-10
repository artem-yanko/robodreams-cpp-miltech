#include "io/gpio_controller.hpp"

#include "utils/logger.hpp"

#include <cerrno>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>

static constexpr auto DROP_PULSE_DURATION = std::chrono::milliseconds(300);

void GpioController::cleanup() {
    if (startRequest != nullptr) {
        gpiod_line_request_release(startRequest);
        startRequest = nullptr;
    }

    if (dropRequest != nullptr) {
        gpiod_line_request_release(dropRequest);
        dropRequest = nullptr;
    }

    if (chip != nullptr) {
        gpiod_chip_close(chip);
        chip = nullptr;
    }
}

GpioController::~GpioController() {
    cleanup();
}

bool GpioController::initSimBank(const std::string& bankPath) {
    simBank = bankPath;
    if (simBank.empty()) {
        ERROR_LOG("SIM bank path is empty");
        return false;
    }

    std::filesystem::path startDir = std::filesystem::path(simBank) / ("sim_gpio" + std::to_string(startLine));
    std::filesystem::path dropDir = std::filesystem::path(simBank) / ("sim_gpio" + std::to_string(dropLine));

    std::error_code error;
    std::filesystem::create_directories(startDir, error);
    if (error) {
        ERROR_LOG("Failed to create sim START directory " << startDir.string() << ": " << error.message());
        return false;
    }

    std::filesystem::create_directories(dropDir, error);
    if (error) {
        ERROR_LOG("Failed to create sim DROP directory " << dropDir.string() << ": " << error.message());
        return false;
    }

    if (!writeSimLine(startLine, 0)) {
        return false;
    }

    if (!writeSimLine(dropLine, 0)) {
        return false;
    }

    return true;
}

bool GpioController::writeSimLine(unsigned offset, int value) {
    std::filesystem::path valuePath = std::filesystem::path(simBank) / ("sim_gpio" + std::to_string(offset)) / "value";
    std::ofstream output(valuePath);
    if (!output.is_open()) {
        ERROR_LOG("Failed to open sim GPIO value file " << valuePath.string());
        return false;
    }

    output << value << '\n';
    output.close();

    return output.good();
}

bool GpioController::requestLine(unsigned offset, enum gpiod_line_value initialValue, gpiod_line_request** request) {
    gpiod_line_settings* lineSettings = gpiod_line_settings_new();
    if (lineSettings == nullptr) {
        return false;
    }

    gpiod_line_config* lineConfig = gpiod_line_config_new();
    if (lineConfig == nullptr) {
        gpiod_line_settings_free(lineSettings);
        return false;
    }

    gpiod_request_config* requestConfig = gpiod_request_config_new();
    if (requestConfig == nullptr) {
        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(lineSettings);
        return false;
    }

    gpiod_line_settings_set_direction(lineSettings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(lineSettings, initialValue);

    if (gpiod_line_config_add_line_settings(lineConfig, &offset, 1, lineSettings) < 0) {
        gpiod_request_config_free(requestConfig);
        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(lineSettings);
        return false;
    }

    gpiod_request_config_set_consumer(requestConfig, "course_project_autopilot");
    *request = gpiod_chip_request_lines(chip, requestConfig, lineConfig);

    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);
    gpiod_line_settings_free(lineSettings);

    return *request != nullptr;
}

bool GpioController::init(const RuntimeConfig& config) {
    cleanup();

    mode = config.mode;
    chipName = config.gpioChip;
    simBank = config.simBank;
    startLine = config.startLine;
    dropLine = config.dropLine;

    if (mode == SIM_MODE) {
        return initSimBank(simBank);
    }

    std::string chipPath = chipName;
    if (!chipPath.empty() && chipPath.rfind("/dev/", 0) != 0) {
        chipPath = "/dev/" + chipPath;
    }

    chip = gpiod_chip_open(chipPath.c_str());
    if (chip == nullptr && chipPath != chipName) {
        chip = gpiod_chip_open(chipName.c_str());
    }

    if (chip == nullptr) {
        ERROR_LOG("Failed to open GPIO chip " << chipPath << ": " << std::strerror(errno));
        cleanup();
        return false;
    }

    if (!requestLine(startLine, GPIOD_LINE_VALUE_ACTIVE, &startRequest)) {
        ERROR_LOG("Failed to request START line " << startLine);
        cleanup();
        return false;
    }

    if (!requestLine(dropLine, GPIOD_LINE_VALUE_INACTIVE, &dropRequest)) {
        ERROR_LOG("Failed to request DROP line " << dropLine);
        cleanup();
        return false;
    }

    return true;
}

bool GpioController::setStartHigh() {
    if (mode == SIM_MODE) {
        return writeSimLine(startLine, 1);
    }

    if (startRequest == nullptr) {
        return false;
    }

    return gpiod_line_request_set_value(startRequest, startLine, GPIOD_LINE_VALUE_ACTIVE) == 0;
}

bool GpioController::pulseDrop() {
    if (mode == SIM_MODE) {
        if (!writeSimLine(dropLine, 1)) {
            return false;
        }

        LOG("DROP line HIGH: sim_gpio" << dropLine);
        std::this_thread::sleep_for(DROP_PULSE_DURATION);
        if (!writeSimLine(dropLine, 0)) {
            return false;
        }

        LOG("DROP line LOW: sim_gpio" << dropLine);
        return true;
    }

    if (dropRequest == nullptr) {
        return false;
    }

    if (gpiod_line_request_set_value(dropRequest, dropLine, GPIOD_LINE_VALUE_ACTIVE) < 0) {
        return false;
    }

    LOG("DROP line HIGH: gpio" << dropLine);
    std::this_thread::sleep_for(DROP_PULSE_DURATION);

    if (gpiod_line_request_set_value(dropRequest, dropLine, GPIOD_LINE_VALUE_INACTIVE) < 0) {
        return false;
    }

    LOG("DROP line LOW: gpio" << dropLine);
    return true;
}
