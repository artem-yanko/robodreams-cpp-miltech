#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

static constexpr uint16_t STUB_PORT = 14560;
static constexpr const char* HEALTHCHECK_PATH = "/tmp/c2_healthy";
static constexpr const char* LOG_PATH = "/var/log/c2/c2.log";

static const char* state_name(C2State state) {
    switch (state) {
    case C2State::DISARMED:
        return "DISARMED";
    case C2State::ARMED_HOLD:
        return "ARMED_HOLD";
    case C2State::ARMED_GUIDED:
        return "ARMED_GUIDED";
    case C2State::ARMED_MANUAL:
        return "ARMED_MANUAL";
    }

    return "UNKNOWN";
}

struct C2Controller::Impl {
    C2State state = C2State::DISARMED;
    FcLink fc;
    UdpSocket udp;
    std::ofstream log_file;
    bool healthcheck_written = false;
    bool hold_sent_in_state = false;

    explicit Impl(uint16_t fc_port)
        : fc(fc_port)
        , udp(STUB_PORT)
        , log_file(LOG_PATH, std::ios::app) {}

    void write_log(const std::string& message) {
        std::cout << message << '\n';
        if (log_file.is_open()) {
            log_file << message << '\n';
            log_file.flush();
        }
    }

    std::optional<std::pair<float, float>> receive_waypoint() {
        char buffer[1024]{};
        ssize_t bytes = udp.recv(buffer, sizeof(buffer) - 1);
        if (bytes <= 0) {
            return std::nullopt;
        }

        buffer[bytes] = '\0';
        const auto payload = nlohmann::json::parse(buffer);
        return std::make_pair(
            payload.at("north_m").get<float>(),
            payload.at("east_m").get<float>());
    }

    void transition(C2State next) {
        if (next == state) {
            return;
        }

        write_log(std::string("[C2] state: ")
            + state_name(state)
            + " -> "
            + state_name(next));
        state = next;
        hold_sent_in_state = false;
    }
};

C2Controller::C2Controller(uint16_t fc_port)
    : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

void C2Controller::tick() {
    if (impl_->fc.is_connected() && !impl_->healthcheck_written) {
        std::ofstream(HEALTHCHECK_PATH).close();
        impl_->healthcheck_written = true;
    }

    if (!impl_->fc.is_armed()) {
        impl_->transition(C2State::DISARMED);
    } else {
        switch (impl_->fc.flight_mode()) {
        case FcLink::FlightMode::Guided:
            impl_->transition(C2State::ARMED_GUIDED);
            break;
        case FcLink::FlightMode::Hold:
            impl_->transition(C2State::ARMED_HOLD);
            break;
        case FcLink::FlightMode::Manual:
            impl_->transition(C2State::ARMED_MANUAL);
            break;
        case FcLink::FlightMode::Unknown:
            impl_->transition(C2State::ARMED_HOLD);
            break;
        }
    }

    if (impl_->state == C2State::ARMED_HOLD && !impl_->hold_sent_in_state) {
        impl_->fc.hold();
        impl_->hold_sent_in_state = true;
    }

    const auto waypoint = impl_->receive_waypoint();
    if (!waypoint.has_value()) {
        return;
    }

    const auto [north_m, east_m] = *waypoint;
    if (impl_->state == C2State::ARMED_GUIDED) {
        impl_->fc.go_to_ned(north_m, east_m);
        impl_->write_log("[C2] fwd: north=" + std::to_string(north_m)
            + " east=" + std::to_string(east_m));
        return;
    }

    impl_->write_log(std::string("[C2] blocked: waypoint in ") + state_name(impl_->state));
}

C2State C2Controller::current_state() const {
    return impl_->state;
}
