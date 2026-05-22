#pragma once

#include "interfaces/ITargetProvider.hpp"

class JsonTargetProvider : public ITargetProvider {
public:
    ~JsonTargetProvider() override;
    bool loadTargets();
    int getTargetCount() override;
    Target getTarget(int index) override;
private:
    TargetData targets{};
    void clearTargets();
};