#include "mavlink/mavlink_gateway.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr uint8_t SYSTEM_ID = 1;
constexpr uint8_t COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;
constexpr double LAT0 = 50.4501;
constexpr double LON0 = 30.5234;
constexpr double METERS_PER_DEGREE = 111320.0;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr auto HEARTBEAT_PERIOD = std::chrono::seconds(1);
constexpr auto TELEMETRY_PERIOD = std::chrono::milliseconds(200);
constexpr auto DROP_ACK_TIMEOUT = std::chrono::milliseconds(500);
constexpr int MAX_DROP_ATTEMPTS = 5;

int32_t scaledDegrees(double degrees) {
    return static_cast<int32_t>(std::llround(degrees * 1e7));
}

int32_t millimeters(double meters) {
    return static_cast<int32_t>(std::llround(meters * 1000.0));
}

int16_t centimetersPerSecond(double metersPerSecond) {
    const double value = std::clamp(metersPerSecond * 100.0, -32768.0, 32767.0);
    return static_cast<int16_t>(std::llround(value));
}

} // namespace

bool MavlinkGateway::init(const std::string& host, uint16_t port) {
    if (!socket.open(host, port)) {
        return false;
    }

    initialized = true;
    LOG("MAVLink UDP enabled: " << host << ":" << port);
    return true;
}

void MavlinkGateway::updateTelemetry(const MissionState& state) {
    if (!initialized || !state.telemetryReceived) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (lastHeartbeat.time_since_epoch().count() == 0 || now - lastHeartbeat >= HEARTBEAT_PERIOD) {
        sendHeartbeat();
        lastHeartbeat = now;
    }

    if (lastTelemetry.time_since_epoch().count() == 0 || now - lastTelemetry >= TELEMETRY_PERIOD) {
        sendGlobalPosition(state);
        sendAttitude(state);
        lastTelemetry = now;
    }
}

void MavlinkGateway::startDropCommand(const MissionState& state, const MissionDecision& decision) {
    if (!initialized || dropCommandActive || dropAckReceived) {
        return;
    }

    GeoPoint dropPoint = localToGeo(decision.dropPointX, decision.dropPointY);
    dropLat = static_cast<float>(dropPoint.lat);
    dropLon = static_cast<float>(dropPoint.lon);
    dropAltitude = state.telemetry.z;
    dropCommandActive = true;
    dropAttempts = 0;
    lastDropAttempt = {};
    sendDropCommand();
}

void MavlinkGateway::pollAck() {
    if (!initialized) {
        return;
    }

    std::vector<uint8_t> data = socket.receiveBytes();
    mavlink_message_t message{};
    mavlink_status_t status{};
    for (uint8_t byte : data) {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &status) && handleAck(message)) {
            return;
        }
    }

    if (!dropCommandActive || dropAckReceived) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - lastDropAttempt >= DROP_ACK_TIMEOUT) {
        if (dropAttempts >= MAX_DROP_ATTEMPTS) {
            ERROR_LOG("MAVLink drop ACK not received after " << dropAttempts << " attempts");
            dropCommandActive = false;
            return;
        }

        sendDropCommand();
    }
}

MavlinkGateway::GeoPoint MavlinkGateway::localToGeo(double x, double y) {
    GeoPoint point{};
    point.lat = LAT0 + y / METERS_PER_DEGREE;
    point.lon = LON0 + x / (METERS_PER_DEGREE * std::cos(LAT0 * DEG_TO_RAD));
    return point;
}

uint16_t MavlinkGateway::headingCentidegrees(double yawRad) {
    double degrees = std::fmod(yawRad * RAD_TO_DEG, 360.0);
    if (degrees < 0.0) {
        degrees += 360.0;
    }

    return static_cast<uint16_t>(std::llround(degrees * 100.0)) % 36000;
}

bool MavlinkGateway::sendMessage(const mavlink_message_t& message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t size = mavlink_msg_to_send_buffer(buffer, &message);
    return socket.sendBytes(buffer, size);
}

void MavlinkGateway::sendHeartbeat() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0,
        0,
        MAV_STATE_ACTIVE
    );
    sendMessage(message);
}

void MavlinkGateway::sendGlobalPosition(const MissionState& state) {
    GeoPoint point = localToGeo(state.telemetry.x, state.telemetry.y);
    mavlink_message_t message{};
    mavlink_msg_global_position_int_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        state.telemetry.t_ms,
        scaledDegrees(point.lat),
        scaledDegrees(point.lon),
        millimeters(state.telemetry.z),
        millimeters(state.telemetry.z),
        centimetersPerSecond(state.telemetry.vy),
        centimetersPerSecond(state.telemetry.vx),
        0,
        headingCentidegrees(state.telemetry.dir)
    );
    sendMessage(message);
}

void MavlinkGateway::sendAttitude(const MissionState& state) {
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        state.telemetry.t_ms,
        0.0f,
        0.0f,
        state.telemetry.dir,
        0.0f,
        0.0f,
        0.0f
    );
    sendMessage(message);
}

void MavlinkGateway::sendDropCommand() {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        SYSTEM_ID,
        COMPONENT_ID,
        MAV_CMD_USER_1,
        static_cast<uint8_t>(dropAttempts),
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        dropLat,
        dropLon,
        dropAltitude
    );

    if (sendMessage(message)) {
        ++dropAttempts;
        lastDropAttempt = std::chrono::steady_clock::now();
        LOG("MAVLink drop command sent, attempt " << dropAttempts);
    }
}

bool MavlinkGateway::handleAck(const mavlink_message_t& message) {
    if (message.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
        return false;
    }

    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    if (ack.command != MAV_CMD_USER_1) {
        return false;
    }

    if (ack.result == MAV_RESULT_ACCEPTED) {
        dropAckReceived = true;
        dropCommandActive = false;
        LOG("MAVLink drop ACK received");
    } else {
        dropCommandActive = false;
        ERROR_LOG("MAVLink drop command rejected, result=" << static_cast<int>(ack.result));
    }

    return true;
}
