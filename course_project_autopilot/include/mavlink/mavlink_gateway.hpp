#pragma once

#include "autopilot/OperatorMode.hpp"
#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"
#include "io/udp_socket.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include <common/mavlink.h>
#include <standard/mavlink_msg_global_position_int.h>

struct MavlinkEvents {
    bool modeChangeRequested{};
    OperatorMode requestedMode{OperatorMode::Manual};
};

class MavlinkGateway {
public:
    bool init(const std::string& host, uint16_t port);

    void updateAutopilotStatus(OperatorMode mode, const char* stateName);
    void updateTelemetry(const MissionState& state);
    void startDropCommand(const MissionState& state, const MissionDecision& decision);
    MavlinkEvents poll();
    void sendModeParam();
    void sendStatusText(uint8_t severity, const std::string& text);

private:
    struct GeoPoint {
        double lat{};
        double lon{};
    };

    static GeoPoint localToGeo(double x, double y);
    static uint16_t headingCentidegrees(double yawRad);
    static float headingRadians(double yawRad);

    bool sendMessage(const mavlink_message_t& message);
    void sendHeartbeat();
    void sendGlobalPosition(const MissionState& state);
    void sendAttitude(const MissionState& state);
    void sendDropCommand();
    bool handleAck(const mavlink_message_t& message);
    bool handleSetMode(const mavlink_message_t& message, MavlinkEvents& events);
    bool handleCommandLong(const mavlink_message_t& message, MavlinkEvents& events);
    bool handleParamRequestList(const mavlink_message_t& message);
    bool handleParamRequestRead(const mavlink_message_t& message);
    bool handleParamSet(const mavlink_message_t& message, MavlinkEvents& events);
    void sendCommandAck(uint16_t command, uint8_t result);

    UdpSocket socket;
    std::chrono::steady_clock::time_point lastHeartbeat{};
    std::chrono::steady_clock::time_point lastTelemetry{};
    std::chrono::steady_clock::time_point lastDropAttempt{};
    bool initialized{};
    bool dropCommandActive{};
    bool dropAckReceived{};
    int dropAttempts{};
    float dropLat{};
    float dropLon{};
    float dropAltitude{};
    OperatorMode autopilotMode{OperatorMode::Manual};
    std::string autopilotStateName{"ManualState"};
};
