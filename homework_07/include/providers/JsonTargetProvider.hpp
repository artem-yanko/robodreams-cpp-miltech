#pragma once

#include "interfaces/ITargetProvider.hpp"
#include <string>

class JsonTargetProvider : public ITargetProvider {
public:
    explicit JsonTargetProvider(const std::string& targetPath = "targets.json");
    ~JsonTargetProvider() override;
    bool loadTargets() override;
    int getTargetCount() const override;
    Target getTarget(int index) const override;

    void step(double targetTimeStep);

private:
    std::string targetPath_;
    void clearTargets();
    void updateCurrentTargets(double targetTimeStep);

    TargetData trajectoryData_{};
    std::vector<Target> currentTargets_{};
    int currentStep_{0};
};
