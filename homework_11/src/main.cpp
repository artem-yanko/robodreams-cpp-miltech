#include "core/MissionProcessor.hpp"
#include "domain/mission_state.hpp"
#include "domain/runtime_config.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "io/drone_link_adapter.hpp"
#include "io/gpio_controller.hpp"
#include "providers/JsonConfigLoader.hpp"
#include "providers/TableSolver.hpp"
#include "utils/logger.hpp"

#include <chrono>
#include <cstring>
#include <memory>
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

static void logDecision(const MissionDecision& decision) {
    LOG("DECISION target=" << decision.targetId
        << ", angleError=" << decision.angleError
        << ", distance=" << decision.distanceToTarget
        << ", accel=" << decision.accel
        << ", turnRate=" << decision.turnRate
        << ", shouldDrop=" << decision.shouldDrop);
}

static RuntimeConfig parseArgs(int argc, char* argv[]) {
    RuntimeConfig config{};
    config.configPath = DEFAULT_CONFIG_DIR "/src/config.json";
    config.ammoPath = DEFAULT_CONFIG_DIR "/src/ammo.json";
    config.ballisticTablePath = DEFAULT_CONFIG_DIR "/data/ballistics/ballistic_table.txt";

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
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config.configPath = argv[++i];
        } else if (std::strcmp(argv[i], "--ammo") == 0 && i + 1 < argc) {
            config.ammoPath = argv[++i];
        } else if (std::strcmp(argv[i], "--ballistic-table") == 0 && i + 1 < argc) {
            config.ballisticTablePath = argv[++i];
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
    std::vector<AmmoParams> ammoList;

    JsonConfigLoader configLoader(config.configPath, config.ammoPath);
    if (!configLoader.loadConfig(config.drone)) {
        ERROR_LOG("Failed to load drone config");
        return 1;
    }

    if (!configLoader.loadAmmo(ammoList)) {
        ERROR_LOG("Failed to load ammo config");
        return 1;
    }

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

    std::unique_ptr<IBallisticSolver> solver = std::make_unique<TableSolver>(config.ballisticTablePath);
    MissionProcessor missionProcessor(config, std::move(solver));

    LOG("Homework 11 skeleton initialized");
    LOG("Mode=" << (config.mode == SIM_MODE ? "sim" : "hw")
        << ", uart=" << config.uartDevice
        << ", gpio=" << config.gpioChip
        << ", simBank=" << (config.simBank.empty() ? "-" : config.simBank)
        << ", config=" << config.configPath
        << ", ammo=" << config.ammoPath
        << ", ballisticTable=" << config.ballisticTablePath
        << ", startLine=" << config.startLine
        << ", dropLine=" << config.dropLine);
    LOG("Config attackSpeed=" << config.drone.attackSpeed
        << ", accelPath=" << config.drone.accelPath
        << ", angularSpeed=" << config.drone.angularSpeed
        << ", turnThreshold=" << config.drone.turnThreshold
        << ", simTimeStep=" << config.drone.simTimeStep);

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
            MissionDecision decision = missionProcessor.update(state);
            logDecision(decision);
            if (!link.sendControl(decision.accel, decision.turnRate)) {
                ERROR_LOG("Failed to send CONTROL");
            }

            if (decision.shouldDrop && !state.dropDone) {
                if (!gpio.pulseDrop()) {
                    ERROR_LOG("Failed to pulse DROP line");
                } else {
                    state.dropDone = true;
                    LOG("DROP triggered");
                }
            }

            lastControlSend = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
