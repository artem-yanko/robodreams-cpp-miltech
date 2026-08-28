#include "io/udp_socket.hpp"

#include "utils/logger.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

UdpSocket::~UdpSocket() {
    if (fd >= 0) {
        close(fd);
    }
}

bool UdpSocket::open(const std::string& host, uint16_t port) {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        ERROR_LOG("Failed to create UDP socket: " << std::strerror(errno));
        return false;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        ERROR_LOG("Failed to set UDP socket non-blocking: " << std::strerror(errno));
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo* addresses = nullptr;
    const std::string portText = std::to_string(port);
    int result = getaddrinfo(host.c_str(), portText.c_str(), &hints, &addresses);
    if (result != 0) {
        ERROR_LOG("Failed to resolve MAVLink UDP host " << host << ": " << gai_strerror(result));
        return false;
    }

    bool connected = false;
    for (addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
        if (connect(fd, address->ai_addr, static_cast<socklen_t>(address->ai_addrlen)) == 0) {
            connected = true;
            break;
        }
    }
    freeaddrinfo(addresses);

    if (!connected) {
        ERROR_LOG("Failed to connect UDP socket to " << host << ":" << port << ": " << std::strerror(errno));
        return false;
    }

    return true;
}

bool UdpSocket::sendBytes(const uint8_t* data, size_t size) {
    ssize_t sent = send(fd, data, size, 0);
    if (sent != static_cast<ssize_t>(size)) {
        ERROR_LOG("Failed to send UDP packet: " << std::strerror(errno));
        return false;
    }

    return true;
}

std::vector<uint8_t> UdpSocket::receiveBytes() {
    std::vector<uint8_t> buffer(2048);
    ssize_t received = recv(fd, buffer.data(), buffer.size(), 0);
    if (received < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ERROR_LOG("Failed to receive UDP packet: " << std::strerror(errno));
        }
        return {};
    }

    buffer.resize(static_cast<size_t>(received));
    return buffer;
}
