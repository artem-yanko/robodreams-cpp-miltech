#include "mavlink/mavlink_gateway.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
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
constexpr auto GCS_HEARTBEAT_TIMEOUT = std::chrono::seconds(3);
constexpr auto TELEMETRY_PERIOD = std::chrono::milliseconds(200);
constexpr auto DROP_ACK_TIMEOUT = std::chrono::milliseconds(500);
constexpr int MAX_DROP_ATTEMPTS = 5;
constexpr uint32_t CPA_MODE_MANUAL = 0;
constexpr uint32_t CPA_MODE_AUTO = 1;
constexpr uint16_t CPA_PARAM_COUNT = 1;
constexpr uint16_t CPA_MODE_PARAM_INDEX = 0;
constexpr const char* CPA_MODE_PARAM_ID = "CPA_MODE";

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

bool paramIdMatches(const char* actual, const char* expected) {
    char buffer[17]{};
    std::memcpy(buffer, actual, 16);
    return std::strcmp(buffer, expected) == 0;
}

// QGC sends integer MAVLink parameters using bytewise encoding in the float field.
int decodeUint8ParamValue(const mavlink_param_set_t& paramSet) {
    uint8_t rawBytes[sizeof(float)]{};
    std::memcpy(rawBytes, &paramSet.param_value, sizeof(rawBytes));
    return static_cast<int>(rawBytes[0]);
}

// Integer MAVLink parameters must be echoed using the same bytewise encoding.
float encodeUint8ParamValue(uint8_t value) {
    float encoded{};
    uint8_t rawBytes[sizeof(float)]{};
    rawBytes[0] = value;
    std::memcpy(&encoded, rawBytes, sizeof(encoded));
    return encoded;
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

void MavlinkGateway::updateAutopilotStatus(OperatorMode mode, const char* stateName) {
    autopilotMode = mode;
    autopilotStateName = stateName != nullptr ? stateName : "UnknownState";
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

MavlinkEvents MavlinkGateway::poll() {
    MavlinkEvents events{};
    if (!initialized) {
        return events;
    }

    std::vector<uint8_t> data = socket.receiveBytes();
    mavlink_message_t message{};
    mavlink_status_t status{};
    for (uint8_t byte : data) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &status)) {
            continue;
        }

        if (handleGcsHeartbeat(message)) {
            continue;
        }

        if (handleAck(message)) {
            continue;
        }

        if (handleSetMode(message, events)) {
            continue;
        }

        if (handleParamRequestList(message)) {
            continue;
        }

        if (handleParamRequestRead(message)) {
            continue;
        }

        if (handleParamSet(message, events)) {
            if (events.modeChangeRequested) {
                return events;
            }
            continue;
        }

        handleCommandLong(message, events);
        if (events.modeChangeRequested) {
            return events;
        }
    }

    if (!dropCommandActive || dropAckReceived) {
        return events;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - lastDropAttempt >= DROP_ACK_TIMEOUT) {
        if (dropAttempts >= MAX_DROP_ATTEMPTS) {
            ERROR_LOG("MAVLink drop ACK not received after " << dropAttempts << " attempts");
            dropCommandActive = false;
            return events;
        }

        sendDropCommand();
    }

    return events;
}

MavlinkGateway::GeoPoint MavlinkGateway::localToGeo(double x, double y) {
    GeoPoint point{};
    point.lat = LAT0 + y / METERS_PER_DEGREE;
    point.lon = LON0 + x / (METERS_PER_DEGREE * std::cos(LAT0 * DEG_TO_RAD));
    return point;
}

uint16_t MavlinkGateway::headingCentidegrees(double yawRad) {
    // Simulator direction uses math angles: 0 rad = East, pi/2 = North.
    // MAVLink heading uses compass angles: 0 deg = North, 90 deg = East.
    double degrees = std::fmod(90.0 - yawRad * RAD_TO_DEG, 360.0);
    if (degrees < 0.0) {
        degrees += 360.0;
    }

    return static_cast<uint16_t>(std::llround(degrees * 100.0)) % 36000;
}

float MavlinkGateway::headingRadians(double yawRad) {
    double radians = std::fmod((PI / 2.0) - yawRad, 2.0 * PI);
    if (radians > PI) {
        radians -= 2.0 * PI;
    }
    if (radians < -PI) {
        radians += 2.0 * PI;
    }

    return static_cast<float>(radians);
}

bool MavlinkGateway::sendMessage(const mavlink_message_t& message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t size = mavlink_msg_to_send_buffer(buffer, &message);
    return socket.sendBytes(buffer, size);
}

void MavlinkGateway::sendHeartbeat() {
    uint8_t baseMode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    uint32_t customMode = CPA_MODE_MANUAL;
    uint8_t systemStatus = MAV_STATE_ACTIVE;

    if (autopilotMode == OperatorMode::Auto) {
        baseMode = MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
        customMode = CPA_MODE_AUTO;
    }

    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        baseMode,
        customMode,
        systemStatus
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
        headingRadians(state.telemetry.dir),
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

bool MavlinkGateway::handleGcsHeartbeat(const mavlink_message_t& message) {
    if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
        return false;
    }

    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    if (heartbeat.type != MAV_TYPE_GCS) {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool connectionRestored = lastGcsHeartbeat.time_since_epoch().count() == 0
        || now - lastGcsHeartbeat > GCS_HEARTBEAT_TIMEOUT;
    lastGcsHeartbeat = now;

    if (connectionRestored) {
        sendModeParam();
        LOG("MAVLink GCS connected: synchronized CPA_MODE="
            << (autopilotMode == OperatorMode::Auto ? CPA_MODE_AUTO : CPA_MODE_MANUAL));
    }

    return true;
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

bool MavlinkGateway::handleSetMode(const mavlink_message_t& message, MavlinkEvents& events) {
    if (message.msgid != MAVLINK_MSG_ID_SET_MODE) {
        return false;
    }

    mavlink_set_mode_t setMode{};
    mavlink_msg_set_mode_decode(&message, &setMode);
    if (setMode.target_system != SYSTEM_ID) {
        return true;
    }

    if (setMode.custom_mode == CPA_MODE_AUTO || (setMode.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) != 0) {
        events.modeChangeRequested = true;
        events.requestedMode = OperatorMode::Auto;
        LOG("MAVLink SET_MODE request: AUTO");
        return true;
    }

    if (setMode.custom_mode == CPA_MODE_MANUAL || (setMode.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) != 0) {
        events.modeChangeRequested = true;
        events.requestedMode = OperatorMode::Manual;
        LOG("MAVLink SET_MODE request: MANUAL");
        return true;
    }

    ERROR_LOG("Unsupported MAVLink SET_MODE custom_mode=" << setMode.custom_mode
        << ", base_mode=" << static_cast<int>(setMode.base_mode));
    return true;
}

bool MavlinkGateway::handleCommandLong(const mavlink_message_t& message, MavlinkEvents& events) {
    if (message.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
        return false;
    }

    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&message, &command);
    if (command.target_system != SYSTEM_ID || command.command != MAV_CMD_DO_SET_MODE) {
        return false;
    }

    const auto baseMode = static_cast<uint8_t>(command.param1);
    const auto customMode = static_cast<uint32_t>(command.param2);
    if (customMode == CPA_MODE_AUTO || (baseMode & MAV_MODE_FLAG_AUTO_ENABLED) != 0) {
        events.modeChangeRequested = true;
        events.requestedMode = OperatorMode::Auto;
        sendCommandAck(command.command, MAV_RESULT_ACCEPTED);
        LOG("MAVLink DO_SET_MODE request: AUTO");
        return true;
    }

    if (customMode == CPA_MODE_MANUAL || (baseMode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) != 0) {
        events.modeChangeRequested = true;
        events.requestedMode = OperatorMode::Manual;
        sendCommandAck(command.command, MAV_RESULT_ACCEPTED);
        LOG("MAVLink DO_SET_MODE request: MANUAL");
        return true;
    }

    sendCommandAck(command.command, MAV_RESULT_UNSUPPORTED);
    ERROR_LOG("Unsupported MAVLink DO_SET_MODE custom_mode=" << customMode
        << ", base_mode=" << static_cast<int>(baseMode));
    return true;
}

void MavlinkGateway::sendCommandAck(uint16_t command, uint8_t result) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        command,
        result,
        0,
        0,
        SYSTEM_ID,
        COMPONENT_ID
    );
    sendMessage(message);
}

bool MavlinkGateway::handleParamRequestList(const mavlink_message_t& message) {
    if (message.msgid != MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
        return false;
    }

    mavlink_param_request_list_t request{};
    mavlink_msg_param_request_list_decode(&message, &request);
    if (request.target_system != SYSTEM_ID) {
        return true;
    }

    if (request.target_component != 0 && request.target_component != COMPONENT_ID) {
        return true;
    }

    sendModeParam();
    LOG("MAVLink PARAM_REQUEST_LIST handled");
    return true;
}

bool MavlinkGateway::handleParamRequestRead(const mavlink_message_t& message) {
    if (message.msgid != MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
        return false;
    }

    mavlink_param_request_read_t request{};
    mavlink_msg_param_request_read_decode(&message, &request);
    if (request.target_system != SYSTEM_ID) {
        return true;
    }

    if (request.target_component != 0 && request.target_component != COMPONENT_ID) {
        return true;
    }

    if (request.param_index == CPA_MODE_PARAM_INDEX || paramIdMatches(request.param_id, CPA_MODE_PARAM_ID)) {
        sendModeParam();
        LOG("MAVLink PARAM_REQUEST_READ handled: CPA_MODE");
        return true;
    }

    ERROR_LOG("Unsupported MAVLink PARAM_REQUEST_READ index=" << request.param_index);
    return true;
}

bool MavlinkGateway::handleParamSet(const mavlink_message_t& message, MavlinkEvents& events) {
    if (message.msgid != MAVLINK_MSG_ID_PARAM_SET) {
        return false;
    }

    mavlink_param_set_t paramSet{};
    mavlink_msg_param_set_decode(&message, &paramSet);
    if (paramSet.target_system != SYSTEM_ID) {
        return true;
    }

    if (paramSet.target_component != 0 && paramSet.target_component != COMPONENT_ID) {
        return true;
    }

    if (!paramIdMatches(paramSet.param_id, CPA_MODE_PARAM_ID)) {
        ERROR_LOG("Unsupported MAVLink PARAM_SET");
        return true;
    }

    uint8_t rawBytes[sizeof(float)]{};
    std::memcpy(rawBytes, &paramSet.param_value, sizeof(rawBytes));
    const int requestedValue = decodeUint8ParamValue(paramSet);
    LOG("MAVLink PARAM_SET CPA_MODE raw: type=" << static_cast<int>(paramSet.param_type)
        << ", float=" << paramSet.param_value
        << ", byte0=" << static_cast<int>(rawBytes[0])
        << ", decoded=" << requestedValue);

    if (requestedValue == static_cast<int>(CPA_MODE_MANUAL)) {
        events.modeChangeRequested = true;
        events.requestedMode = OperatorMode::Manual;
        LOG("MAVLink PARAM_SET CPA_MODE=MANUAL");
        return true;
    }

    if (requestedValue == static_cast<int>(CPA_MODE_AUTO)) {
        events.modeChangeRequested = true;
        events.requestedMode = OperatorMode::Auto;
        LOG("MAVLink PARAM_SET CPA_MODE=AUTO");
        return true;
    }

    sendModeParam();
    ERROR_LOG("Unsupported MAVLink PARAM_SET CPA_MODE value=" << requestedValue);
    return true;
}

void MavlinkGateway::sendModeParam() {
    const auto modeValue = static_cast<uint8_t>(autopilotMode == OperatorMode::Auto ? CPA_MODE_AUTO : CPA_MODE_MANUAL);
    mavlink_message_t message{};
    mavlink_msg_param_value_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        CPA_MODE_PARAM_ID,
        encodeUint8ParamValue(modeValue),
        MAV_PARAM_TYPE_UINT8,
        CPA_PARAM_COUNT,
        CPA_MODE_PARAM_INDEX
    );
    sendMessage(message);
}

void MavlinkGateway::sendStatusText(uint8_t severity, const std::string& text) {
    if (!initialized) {
        return;
    }

    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN]{};
    std::strncpy(buffer, text.c_str(), sizeof(buffer) - 1);

    mavlink_message_t message{};
    mavlink_msg_statustext_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        severity,
        buffer,
        0,
        0
    );
    sendMessage(message);
}
