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

bool DroneLinkAdapter::pollIncoming() {
    return uartFd >= 0;
}

bool DroneLinkAdapter::sendControl(float accel, float turnRate) {
    (void)accel;
    (void)turnRate;
    return uartFd >= 0;
}
