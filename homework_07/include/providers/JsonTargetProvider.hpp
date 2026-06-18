#pragma once

#include "interfaces/ITargetProvider.hpp"
#include <atomic>
#include <deque>
#include <mutex>
#include <string>

class JsonTargetProvider : public ITargetProvider {
public:
    explicit JsonTargetProvider(const std::string& targetPath = "targets.json");

    void run(double targetTimeStep, double arrayTimeStep, double timeScale) override;
    void start() override;
    void stop() override;
    bool isThreadReady() const override;
    double getCurrentSimTime() const override;
    void setAllowedSimTime(double simTime) override;

    ~JsonTargetProvider() override;
    bool loadTargets() override;
    int getTargetCount() const override;
    Target getTarget(int index) const override;
    Target getTargetAt(int index, double simTime) const override;
    void step(double targetTimeStep, double arrayTimeStep) override;

private:
    struct TargetSnapshot {
        double simTime{};
        std::vector<Target> targets{};
    };

    std::string targetPath_;
    void clearTargets();
    void updateCurrentTargets(double targetTimeStep, double arrayTimeStep);
    Coord interpolateTargetPosition(int targetIndex, double time, double arrayTimeStep) const;

    TargetData trajectoryData_{};
    std::vector<Target> currentTargets_{};
    double currentTime_{0.0};
    std::deque<TargetSnapshot> targetHistory_{};
    mutable std::mutex currentTargetsMutex_{};
    std::atomic<bool> running_{true};
    std::atomic<bool> started_{false};
    std::atomic<bool> threadReady_{false};
    std::atomic<double> allowedSimTime_{0.0};
};
