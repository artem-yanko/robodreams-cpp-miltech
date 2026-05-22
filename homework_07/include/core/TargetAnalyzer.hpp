#pragma once

#include "domain/types.hpp"

class TargetAnalyzer {
public:
    TargetAnalyzer();
    bool getTarget();
    ~TargetAnalyzer();

private:
    void clearAnalysis();

    Target targets;
};