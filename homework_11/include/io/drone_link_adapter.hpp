#pragma once

#include <string>

class DroneLinkAdapter {
public:
    ~DroneLinkAdapter();

    bool open(const std::string& uartDevice);
    bool pollIncoming();
    bool sendControl(float accel, float turnRate);

private:
    int uartFd{-1};
};
