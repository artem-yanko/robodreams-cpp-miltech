#include "publishing/ResultPublisher.hpp"

#include "utils/json.hpp"
#include "utils/logger.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <netdb.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace {

class SocketHandle {
public:
    explicit SocketHandle(int fd) : fd(fd) {}
    ~SocketHandle() {
        if (fd >= 0) {
            close(fd);
        }
    }

    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;

    int get() const {
        return fd;
    }

private:
    int fd;
};

bool setNonBlocking(int fd, bool nonBlocking) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }

    if (nonBlocking) {
        flags |= O_NONBLOCK;
    } else {
        flags &= ~O_NONBLOCK;
    }

    return fcntl(fd, F_SETFL, flags) == 0;
}

bool waitForWritable(int fd, int timeoutSec) {
    fd_set writeSet;
    FD_ZERO(&writeSet);
    FD_SET(fd, &writeSet);

    timeval timeout{};
    timeout.tv_sec = timeoutSec;

    int result = select(fd + 1, nullptr, &writeSet, nullptr, &timeout);
    return result > 0 && FD_ISSET(fd, &writeSet);
}

bool connectWithTimeout(int fd, const sockaddr* address, socklen_t addressLen, int timeoutSec) {
    if (!setNonBlocking(fd, true)) {
        return false;
    }

    int result = connect(fd, address, addressLen);
    if (result == 0) {
        return setNonBlocking(fd, false);
    }

    if (errno != EINPROGRESS) {
        return false;
    }

    if (!waitForWritable(fd, timeoutSec)) {
        errno = ETIMEDOUT;
        return false;
    }

    int socketError = 0;
    socklen_t socketErrorLen = sizeof(socketError);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socketError, &socketErrorLen) != 0 || socketError != 0) {
        errno = socketError;
        return false;
    }

    return setNonBlocking(fd, false);
}

std::string readTextFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open " + path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string statusText(int statusCode) {
    if (statusCode >= 200 && statusCode < 300) {
        return "success";
    }
    if (statusCode == 400) {
        return "bad-request";
    }
    if (statusCode == 401) {
        return "unauthorized";
    }
    if (statusCode == 503) {
        return "temporary-unavailable";
    }
    return "http-" + std::to_string(statusCode);
}

} // namespace

ResultPublisher::ResultPublisher(PublishConfig config)
    : config(std::move(config)) {
}

PublishReport ResultPublisher::publishFile(const std::string& simulationPath) const {
    PublishReport report{};
    report.testId = config.testId;

    nlohmann::ordered_json requestBody;
    try {
        requestBody["studentId"] = config.studentId;
        requestBody["testId"] = config.testId;
        requestBody["simulation"] = nlohmann::ordered_json::parse(readTextFile(simulationPath));
    } catch (const std::exception& exception) {
        report.status = std::string("local-error: ") + exception.what();
        return report;
    }

    const std::string body = requestBody.dump();
    for (int attempt = 1; attempt <= config.maxAttempts; ++attempt) {
        report.attempts = attempt;
        HttpResponse response = request("POST", "/api/dz12/results", body);

        if (response.transportError || response.timedOut) {
            report.status = response.timedOut ? "timeout" : "transport-error";
        } else if (response.statusCode >= 200 && response.statusCode < 300) {
            report.status = verifyResult() ? "published" : "post-ok-get-missing";
            return report;
        } else {
            report.status = statusText(response.statusCode);
            if (response.statusCode == 400 || response.statusCode == 401) {
                ERROR_LOG("Publish failed for " << config.testId << ": " << response.body);
                return report;
            }
        }

        const bool canRetry = response.timedOut || response.transportError || response.statusCode == 503;
        if (!canRetry || attempt == config.maxAttempts) {
            return report;
        }

        std::this_thread::sleep_for(std::chrono::seconds(config.retryDelaySec));
    }

    return report;
}

bool ResultPublisher::verifyResult() const {
    const std::string path = "/api/dz12/results/" + config.testId + "/" + config.studentId;
    HttpResponse response = request("GET", path, "");
    if (response.transportError || response.timedOut || response.statusCode != 200) {
        return false;
    }

    try {
        nlohmann::json body = nlohmann::json::parse(response.body);
        return body.value("found", false);
    } catch (const std::exception&) {
        return false;
    }
}

ResultPublisher::HttpResponse ResultPublisher::request(
    const std::string& method,
    const std::string& path,
    const std::string& body) const {
    HttpResponse response{};

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* addresses = nullptr;
    const std::string port = std::to_string(config.port);
    int resolveResult = getaddrinfo(config.host.c_str(), port.c_str(), &hints, &addresses);
    if (resolveResult != 0) {
        response.transportError = true;
        response.error = gai_strerror(resolveResult);
        return response;
    }

    int connectedFd = -1;
    for (addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
        int fd = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (fd < 0) {
            continue;
        }

        if (connectWithTimeout(fd, address->ai_addr, static_cast<socklen_t>(address->ai_addrlen), config.timeoutSec)) {
            connectedFd = fd;
            break;
        }

        close(fd);
    }
    freeaddrinfo(addresses);

    if (connectedFd < 0) {
        response.transportError = true;
        response.timedOut = errno == ETIMEDOUT;
        response.error = std::strerror(errno);
        return response;
    }

    SocketHandle socket(connectedFd);
    timeval timeout{};
    timeout.tv_sec = config.timeoutSec;
    setsockopt(socket.get(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket.get(), SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    std::ostringstream requestStream;
    requestStream << method << " " << path << " HTTP/1.1\r\n"
                  << "Host: " << config.host << "\r\n"
                  << "Connection: close\r\n"
                  << "x-api-key: " << config.apiKey << "\r\n";
    if (method == "POST") {
        requestStream << "Content-Type: application/json\r\n"
                      << "Content-Length: " << body.size() << "\r\n";
    }
    requestStream << "\r\n" << body;

    const std::string requestText = requestStream.str();
    const char* data = requestText.data();
    size_t bytesLeft = requestText.size();
    while (bytesLeft > 0) {
        ssize_t sent = send(socket.get(), data, bytesLeft, 0);
        if (sent < 0) {
            response.transportError = true;
            response.timedOut = errno == EAGAIN || errno == EWOULDBLOCK;
            response.error = std::strerror(errno);
            return response;
        }

        data += sent;
        bytesLeft -= static_cast<size_t>(sent);
    }

    std::string rawResponse;
    char buffer[4096];
    while (true) {
        ssize_t received = recv(socket.get(), buffer, sizeof(buffer), 0);
        if (received == 0) {
            break;
        }
        if (received < 0) {
            response.transportError = true;
            response.timedOut = errno == EAGAIN || errno == EWOULDBLOCK;
            response.error = std::strerror(errno);
            return response;
        }

        rawResponse.append(buffer, static_cast<size_t>(received));
    }

    const size_t statusEnd = rawResponse.find("\r\n");
    if (statusEnd == std::string::npos) {
        response.transportError = true;
        response.error = "invalid http response";
        return response;
    }

    std::istringstream statusLine(rawResponse.substr(0, statusEnd));
    std::string httpVersion;
    statusLine >> httpVersion >> response.statusCode;

    const size_t bodyStart = rawResponse.find("\r\n\r\n");
    if (bodyStart != std::string::npos) {
        response.body = rawResponse.substr(bodyStart + 4);
    }

    return response;
}
