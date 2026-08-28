#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket();

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    bool open(const std::string& host, uint16_t port);
    bool sendBytes(const uint8_t* data, size_t size);
    std::vector<uint8_t> receiveBytes();

private:
    int fd{-1};
};
