#include "simulation/SimulationRecorder.hpp"

#include "utils/json.hpp"
#include "utils/logger.hpp"

#include <filesystem>
#include <fstream>
#include <utility>

SimulationRecorder::SimulationRecorder(std::string outputPath)
    : outputFilePath(std::move(outputPath)) {
}

void SimulationRecorder::record(const MissionState& state, const MissionDecision& decision) {
    if (!state.telemetryReceived) {
        return;
    }

    if (hasLastTelemetry && state.telemetry.t_ms == lastTelemetryMs) {
        return;
    }

    lastTelemetryMs = state.telemetry.t_ms;
    hasLastTelemetry = true;

    SimulationStep step{};
    step.positionX = state.telemetry.x;
    step.positionY = state.telemetry.y;
    step.direction = state.telemetry.dir;
    step.state = static_cast<int>(state.telemetry.state);
    step.targetIndex = decision.targetId;
    step.dropPointX = decision.dropPointX;
    step.dropPointY = decision.dropPointY;
    step.aimPointX = decision.impactPointX;
    step.aimPointY = decision.impactPointY;
    step.predictedTargetX = decision.predictedTargetX;
    step.predictedTargetY = decision.predictedTargetY;
    step.timeSecSinceStart = static_cast<double>(state.telemetry.t_ms) / 1000.0;
    steps.push_back(step);
}

bool SimulationRecorder::write() const {
    std::filesystem::path path(outputFilePath);
    std::error_code error;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            ERROR_LOG("Failed to create simulation output directory: " << error.message());
            return false;
        }
    }

    nlohmann::ordered_json outputJson;
    outputJson["totalSteps"] = steps.empty() ? 0 : static_cast<int>(steps.size()) - 1;
    outputJson["steps"] = nlohmann::ordered_json::array();

    for (const SimulationStep& step : steps) {
        nlohmann::ordered_json stepJson;
        stepJson["position"] = {
            {"x", step.positionX},
            {"y", step.positionY}
        };
        stepJson["direction"] = step.direction;
        stepJson["state"] = step.state;
        stepJson["targetIndex"] = step.targetIndex;
        stepJson["dropPoint"] = {
            {"x", step.dropPointX},
            {"y", step.dropPointY}
        };
        stepJson["aimPoint"] = {
            {"x", step.aimPointX},
            {"y", step.aimPointY}
        };
        stepJson["predictedTarget"] = {
            {"x", step.predictedTargetX},
            {"y", step.predictedTargetY}
        };
        stepJson["timeSecSinceStart"] = step.timeSecSinceStart;
        outputJson["steps"].push_back(stepJson);
    }

    const std::filesystem::path tempPath = path.string() + ".tmp";
    std::ofstream outputFile(tempPath);
    if (!outputFile) {
        ERROR_LOG("Failed to open simulation output: " << tempPath.string());
        return false;
    }

    outputFile << outputJson.dump(2);
    outputFile.close();
    if (!outputFile) {
        ERROR_LOG("Failed to write simulation output: " << tempPath.string());
        return false;
    }

    std::filesystem::rename(tempPath, path, error);
    if (error) {
        ERROR_LOG("Failed to move simulation output into place: " << error.message());
        return false;
    }

    return true;
}

const std::string& SimulationRecorder::outputPath() const {
    return outputFilePath;
}
