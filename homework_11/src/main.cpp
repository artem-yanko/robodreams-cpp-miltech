#include "core/ComponentFactory.hpp"
#include "core/MissionProcessor.hpp"
#include "domain/mission_state.hpp"
#include "domain/runtime_config.hpp"
#include "interfaces/IBallisticSolver.hpp"
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

static void logDroneCfg(const MissionState& state) {
    LOG("DRONE_CFG attackSpeed=" << state.droneCfg.attackSpeed
        << ", accelerationPath=" << state.droneCfg.accelerationPath
        << ", angularSpeed=" << state.droneCfg.angularSpeed
        << ", turnThreshold=" << state.droneCfg.turnThreshold
        << ", timeStep=" << state.droneCfg.timeStep
        << ", timeScale=" << state.droneCfg.timeScale);
}

static void logTelemetry(const MissionState& state) {
    LOG("TELEMETRY t_ms=" << state.telemetry.t_ms
        << ", pos=(" << state.telemetry.x << ", " << state.telemetry.y << ", " << state.telemetry.z << ")"
        << ", speed=" << state.telemetry.speed
        << ", dir=" << state.telemetry.dir
        << ", state=" << static_cast<int>(state.telemetry.state));
}

static void logTarget(const MissionState& state) {
    (void)state;
    DEBUG("TARGET id=" << static_cast<int>(state.lastTargetUpdate.id)
          << ", pos=(" << state.lastTargetUpdate.x << ", " << state.lastTargetUpdate.y << ")"
          << ", knownTargets=" << state.targets.size());
}

static void logDecision(const MissionDecision& decision) {
    LOG("DECISION target=" << decision.targetId
        << ", angleError=" << decision.angleError
        << ", distance=" << decision.distanceToTarget
        << ", predictedTarget=(" << decision.predictedTargetX << ", " << decision.predictedTargetY << ")"
        << ", dropPoint=(" << decision.dropPointX << ", " << decision.dropPointY << ")"
        << ", distanceToDropPoint=" << decision.distanceToDropPoint
        << ", releaseHeading=" << decision.releaseHeading
        << ", releaseTurnThreshold=" << decision.releaseTurnThreshold
        << ", accel=" << decision.accel
        << ", turnRate=" << decision.turnRate
        << ", shouldDrop=" << decision.shouldDrop);
}

static void applyDroneCfg(DroneConfig& drone, const MissionState& state) {
    if (!state.droneCfgReceived) {
        return;
    }

    drone.attackSpeed = state.droneCfg.attackSpeed;
    drone.accelPath = state.droneCfg.accelerationPath;
    drone.angularSpeed = state.droneCfg.angularSpeed;
    drone.turnThreshold = state.droneCfg.turnThreshold;
    drone.simTimeStep = state.droneCfg.timeStep;
    drone.physicsTimeStep = state.droneCfg.timeStep;
    drone.timeScale = state.droneCfg.timeScale;
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

    std::unique_ptr<JsonConfigLoader> configLoader = ComponentFactory::createLoader(config.configPath, config.ammoPath);
    if (!configLoader || !configLoader->loadConfig(config.drone)) {
        ERROR_LOG("Failed to load drone config");
        return 1;
    }

    if (!configLoader->loadAmmo(ammoList)) {
        ERROR_LOG("Failed to load ammo config");
        return 1;
    }

    std::unique_ptr<DroneLinkAdapter> link = ComponentFactory::createDroneLinkAdapter();
    if (!link || !link->open(config.uartDevice)) {
        ERROR_LOG("Failed to open UART link");
        return 1;
    }

    std::unique_ptr<GpioController> gpio = ComponentFactory::createGpioController();
    if (!gpio || !gpio->init(config)) {
        ERROR_LOG("Failed to initialize GPIO");
        return 1;
    }

    if (!gpio->setStartHigh()) {
        ERROR_LOG("Failed to raise START line");
        return 1;
    }
    state.startRaised = true;

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
    bool droneCfgLogged = false;
    bool runtimeConfigApplied = false;
    uint32_t lastTelemetryLogMs = 0;
    std::chrono::steady_clock::time_point startup = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastControlSend = std::chrono::steady_clock::now();
    const auto controlPeriod = std::chrono::milliseconds(20);
    std::unique_ptr<MissionProcessor> missionProcessor;

    while (true) {
        int packets = link->pollIncoming(state);
        if (packets < 0) {
            ERROR_LOG("UART polling failed");
            return 1;
        }

        if (state.ammoReceived && !ammoLogged) {
            logAmmo(state);
            ammoLogged = true;
        }

        if (state.droneCfgReceived && !droneCfgLogged) {
            logDroneCfg(state);
            droneCfgLogged = true;
        }

        if (!runtimeConfigApplied && state.telemetryReceived) {
            const bool haveRuntimeCfg = state.droneCfgReceived;
            const bool startupGraceElapsed = std::chrono::steady_clock::now() - startup >= std::chrono::milliseconds(500);
            if (haveRuntimeCfg || startupGraceElapsed) {
                applyDroneCfg(config.drone, state);
                config.drone.startPos = Coord{state.telemetry.x, state.telemetry.y};
                config.drone.altitude = state.telemetry.z;
                config.drone.initialDir = state.telemetry.dir;
                runtimeConfigApplied = true;

                LOG("Runtime config attackSpeed=" << config.drone.attackSpeed
                    << ", accelPath=" << config.drone.accelPath
                    << ", angularSpeed=" << config.drone.angularSpeed
                    << ", turnThreshold=" << config.drone.turnThreshold
                    << ", simTimeStep=" << config.drone.simTimeStep
                    << ", startPos=(" << config.drone.startPos.x << ", " << config.drone.startPos.y << ")"
                    << ", altitude=" << config.drone.altitude
                    << ", initialDir=" << config.drone.initialDir);

                std::unique_ptr<IBallisticSolver> solver =
                    ComponentFactory::createSolver(SolverType::TABLE, config.ballisticTablePath);
                if (!solver) {
                    ERROR_LOG("Failed to create ballistic solver");
                    return 1;
                }

                missionProcessor = ComponentFactory::createMissionProcessor(config, std::move(solver));
                if (!missionProcessor) {
                    ERROR_LOG("Failed to create mission processor");
                    return 1;
                }
            }
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
        if (missionProcessor && now - lastControlSend >= controlPeriod) {
            MissionDecision decision = missionProcessor->update(state);
            logDecision(decision);
            if (!link->sendControl(decision.accel, decision.turnRate)) {
                ERROR_LOG("Failed to send CONTROL");
            }

            if (decision.shouldDrop && !state.dropDone) {
                if (!gpio->pulseDrop()) {
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
