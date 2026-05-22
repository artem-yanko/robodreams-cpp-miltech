#pragma once

#include "interfaces/ITargetProvider.hpp"

class JsonTargetProvider : public ITargetProvider {
public:
    ~JsonTargetProvider() override;
    bool loadTargets() override;
    int getTargetCount() override;
    const TargetData& getTargetsData() override;

private:
    void clearTargets();

    TargetData targets{};
};