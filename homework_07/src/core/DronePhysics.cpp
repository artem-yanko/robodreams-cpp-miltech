#include "core/DronePhysics.hpp"

DronePhysics::DronePhysics(const DroneConfig& config)
    : config_(config) {
    telemetry_.pos = config.startPos;
    telemetry_.direction = config.initialDir;
    telemetry_.mode = DroneMode::Stopped;
}

void DronePhysics::submitCommand(const DroneCommand& command) {
    commandQueue_.push(command);
}

DroneTelemetry DronePhysics::getTelemetry() const {
    return telemetry_;
}

void DronePhysics::step() {
    DroneCommand nextCommand{};
    bool hasCommand = false;

    while (commandQueue_.tryPop(nextCommand)) {
        currentCommand_ = nextCommand;
        hasCommand = true;
    }

    if (hasCommand) {
        telemetry_.mode = currentCommand_.mode;
    }
}
