#include "io/drone_link_adapter.hpp"

#include "utils/logger.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static bool configureRawUart(int fd) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);

    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

DroneLinkAdapter::~DroneLinkAdapter() {
    if (uartFd >= 0) {
        close(uartFd);
    }
}

bool DroneLinkAdapter::open(const std::string& uartDevice) {
    uartFd = ::open(uartDevice.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uartFd < 0) {
        ERROR_LOG("Failed to open UART " << uartDevice << ": " << std::strerror(errno));
        return false;
    }

    if (!configureRawUart(uartFd)) {
        ERROR_LOG("Failed to configure UART " << uartDevice);
        close(uartFd);
        uartFd = -1;
        return false;
    }

    return true;
}

void DroneLinkAdapter::configureTargetLoss(uint32_t afterMs, uint32_t durationMs) {
    debugTargetLossEnabled = true;
    debugTargetLossAfterMs = afterMs;
    debugTargetLossDurationMs = durationMs;
}

int DroneLinkAdapter::pollIncoming(MissionState& state) {
    if (uartFd < 0) {
        return -1;
    }

    uint8_t input[256]{};
    ssize_t n = read(uartFd, input, sizeof(input));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        ERROR_LOG("UART read failed: " << std::strerror(errno));
        return -1;
    }

    if (n == 0) {
        return 0;
    }

    uint8_t type = 0;
    uint8_t len = 0;
    uint8_t payload[260]{};
    int packets = 0;

    for (ssize_t i = 0; i < n; ++i) {
        if (!parser.feed(input[i], type, payload, len)) {
            continue;
        }

        ++packets;

        if (type == dlink::PKT_TELEMETRY && len == sizeof(dlink::Telemetry)) {
            std::memcpy(&state.telemetry, payload, sizeof(dlink::Telemetry));
            state.telemetryReceived = true;
        } else if (type == dlink::PKT_AMMO && len == sizeof(dlink::AmmoCfg)) {
            std::memcpy(&state.ammo, payload, sizeof(dlink::AmmoCfg));
            state.ammoReceived = true;
        } else if (type == dlink::PKT_CONFIG && len == sizeof(dlink::DroneCfg)) {
            std::memcpy(&state.droneCfg, payload, sizeof(dlink::DroneCfg));
            state.droneCfgReceived = true;
        } else if (type == dlink::PKT_CONFIG) {
        } else if (type == dlink::PKT_TARGET && len == sizeof(dlink::TargetPos)) {
            if (shouldIgnoreTarget(state.telemetry.t_ms)) {
                continue;
            }

            dlink::TargetPos target{};
            std::memcpy(&target, payload, sizeof(dlink::TargetPos));
            state.updateTarget(target, state.telemetry.t_ms);
        }
    }

    return packets;
}

bool DroneLinkAdapter::shouldIgnoreTarget(uint32_t telemetryTimeMs) {
    bool shouldIgnore = false;
    if (debugTargetLossEnabled && telemetryTimeMs >= debugTargetLossAfterMs) {
        const uint32_t elapsed = telemetryTimeMs - debugTargetLossAfterMs;
        shouldIgnore = debugTargetLossDurationMs == 0 || elapsed < debugTargetLossDurationMs;
    }

    if (shouldIgnore && !debugTargetLossActive) {
        debugTargetLossActive = true;
        LOG("DEBUG TARGET LOSS started at t_ms=" << telemetryTimeMs);
    } else if (!shouldIgnore && debugTargetLossActive) {
        debugTargetLossActive = false;
        LOG("DEBUG TARGET LOSS ended at t_ms=" << telemetryTimeMs);
    }

    return shouldIgnore;
}

bool DroneLinkAdapter::sendControl(float accel, float turnRate) {
    if (uartFd < 0) {
        return false;
    }

    dlink::Control control{accel, turnRate};
    uint8_t output[64]{};
    size_t size = dlink::encode(dlink::PKT_CONTROL, &control, sizeof(control), output);
    ssize_t written = write(uartFd, output, size);
    if (written < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return true;
        }
        ERROR_LOG("UART write failed: " << std::strerror(errno));
        return false;
    }

    return written == static_cast<ssize_t>(size);
}
