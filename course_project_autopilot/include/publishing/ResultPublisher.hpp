#pragma once

#include <string>

struct PublishConfig {
    std::string host{"cppmiltech.com.ua"};
    std::string apiKey{"dz12-vX7mK4qT9r2w"};
    std::string studentId{"1035"};
    std::string testId{};
    int port{80};
    int maxAttempts{5};
    int timeoutSec{2};
    int retryDelaySec{1};
};

struct PublishReport {
    std::string testId{};
    std::string status{};
    int attempts{};
};

class ResultPublisher {
public:
    explicit ResultPublisher(PublishConfig config);

    PublishReport publishFile(const std::string& simulationPath) const;

private:
    struct HttpResponse {
        int statusCode{};
        std::string body{};
        bool timedOut{};
        bool transportError{};
        std::string error{};
    };

    HttpResponse request(const std::string& method, const std::string& path, const std::string& body) const;
    bool verifyResult() const;

    PublishConfig config;
};
