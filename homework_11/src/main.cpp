#include "domain/mission_state.hpp"
#include "domain/runtime_config.hpp"
#include "io/drone_link_adapter.hpp"
#include "io/gpio_controller.hpp"
#include "utils/logger.hpp"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

static void logAmmo(const MissionState& state) {
    LOG("AMMO name=" << state.ammo.name
        << ", mass=" << state.ammo.mass
        << ", drag=" << state.ammo.drag
        << ", lift=" << state.ammo.lift
        << ", hitRadius=" << state.ammo.hitRadius
        << ", nTargets=" << static_cast<int>(state.ammo.nTargets));
}

static void logTelemetry(const MissionState& state) {
    LOG("TELEMETRY t_ms=" << state.telemetry.t_ms
        << ", pos=(" << state.telemetry.x << ", " << state.telemetry.y << ", " << state.telemetry.z << ")"
        << ", speed=" << state.telemetry.speed
        << ", dir=" << state.telemetry.dir
        << ", state=" << static_cast<int>(state.telemetry.state));
}

static void logTarget(const MissionState& state) {
    DEBUG("TARGET id=" << static_cast<int>(state.lastTargetUpdate.id)
          << ", pos=(" << state.lastTargetUpdate.x << ", " << state.lastTargetUpdate.y << ")"
          << ", knownTargets=" << state.targets.size());
}

static RuntimeConfig parseArgs(int argc, char* argv[]) {
    RuntimeConfig config{};

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--sim") == 0) {
            config.mode = SIM_MODE;
            config.uartDevice = "/tmp/ttyA";
            config.gpioChip = "gpiochip1";
            config.simBank = "/tmp/dz22-bank";
            config.startLine = 24;
            config.dropLine = 23;
        } else if (std::strcmp(argv[i], "--hw") == 0) {
            config.mode = HW_MODE;
            config.uartDevice = "/dev/ttyAMA1";
            config.gpioChip = "gpiochip0";
            config.simBank.clear();
            config.startLine = 27;
            config.dropLine = 22;
        } else if (std::strcmp(argv[i], "--uart") == 0 && i + 1 < argc) {
            config.uartDevice = argv[++i];
        } else if (std::strcmp(argv[i], "--gpiochip") == 0 && i + 1 < argc) {
            config.gpioChip = argv[++i];
        } else if (std::strcmp(argv[i], "--sim-bank") == 0 && i + 1 < argc) {
            config.simBank = argv[++i];
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
        << ", simBank=" << (config.simBank.empty() ? "-" : config.simBank)
        << ", startLine=" << config.startLine
        << ", dropLine=" << config.dropLine);

    bool ammoLogged = false;
    uint32_t lastTelemetryLogMs = 0;
    std::chrono::steady_clock::time_point lastControlSend = std::chrono::steady_clock::now();

    while (true) {
        int packets = link.pollIncoming(state);
        if (packets < 0) {
            ERROR_LOG("UART polling failed");
            return 1;
        }

        if (state.ammoReceived && !ammoLogged) {
            logAmmo(state);
            ammoLogged = true;
        }

        if (state.telemetryReceived && state.telemetry.t_ms >= lastTelemetryLogMs + 1000) {
            logTelemetry(state);
            lastTelemetryLogMs = state.telemetry.t_ms;
        }

        if (state.targetUpdateReceived) {
            logTarget(state);
            state.targetUpdateReceived = false;
        }

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (now - lastControlSend >= std::chrono::milliseconds(100)) {
            if (!link.sendControl(0.0f, 0.0f)) {
                ERROR_LOG("Failed to send CONTROL");
            }
            lastControlSend = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
