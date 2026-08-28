#pragma once

#include "domain/mission_decision.hpp"
#include "domain/mission_state.hpp"
#include "io/udp_socket.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include <common/mavlink.h>
#include <standard/mavlink_msg_global_position_int.h>

class MavlinkGateway {
public:
    bool init(const std::string& host, uint16_t port);

    void updateTelemetry(const MissionState& state);
    void startDropCommand(const MissionState& state, const MissionDecision& decision);
    void pollAck();

private:
    struct GeoPoint {
        double lat{};
        double lon{};
    };

    static GeoPoint localToGeo(double x, double y);
    static uint16_t headingCentidegrees(double yawRad);

    bool sendMessage(const mavlink_message_t& message);
    void sendHeartbeat();
    void sendGlobalPosition(const MissionState& state);
    void sendAttitude(const MissionState& state);
    void sendDropCommand();
    bool handleAck(const mavlink_message_t& message);

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
};
