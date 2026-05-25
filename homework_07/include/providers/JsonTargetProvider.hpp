#pragma once

#include "interfaces/ITargetProvider.hpp"
#include <string>

class JsonTargetProvider : public ITargetProvider {
public:
    explicit JsonTargetProvider(const std::string& targetPath = "targets.json");
    ~JsonTargetProvider() override;
    bool loadTargets() override;
    int getTargetCount() override;
    const TargetData& getTargetsData() override;

private:
    std::string targetPath_;
    void clearTargets();

    TargetData targets{};
};
