#pragma once

#include "domain/types.hpp"

class TargetAnalyzer {
public:
     Target analyzeTarget(int targetIndex, const TargetData& targets, double simulationTime, double arrayTimeStep);
};