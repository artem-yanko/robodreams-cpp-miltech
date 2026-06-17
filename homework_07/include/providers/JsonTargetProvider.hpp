#pragma once

#include "interfaces/ITargetProvider.hpp"
#include <string>
#include <atomic>
#include <mutex>

class JsonTargetProvider : public ITargetProvider {
public:
    explicit JsonTargetProvider(const std::string& targetPath = "targets.json");

    void run(double targetTimeStep, double arrayTimeStep, double timeScale) override;
    void start() override;
    void stop() override;
    bool isThreadReady() const override;

    ~JsonTargetProvider() override;
    bool loadTargets() override;
    int getTargetCount() const override;
    Target getTarget(int index) const override;
    void step(double targetTimeStep, double arrayTimeStep) override;

private:
    std::string targetPath_;
    void clearTargets();
    void updateCurrentTargets(double targetTimeStep, double arrayTimeStep);
    Coord interpolateTargetPosition(int targetIndex, double time, double arrayTimeStep) const;

    TargetData trajectoryData_{};
    std::vector<Target> currentTargets_{};
    double currentTime_{0.0};
    mutable std::mutex currentTargetsMutex_{};
    std::atomic<bool> running_{true};
    std::atomic<bool> started_{false};
    std::atomic<bool> threadReady_{false};
};
