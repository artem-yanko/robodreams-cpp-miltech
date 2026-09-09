#pragma once

#include "domain/types.hpp"

double length(const Coord& c);
double distanceBetween(const Coord& from, const Coord& to);
Coord normalize(const Coord& c);
double calculateAngleDifference(double currentDir, double targetDir);
double calculateDroneAcceleration(double attackSpeed, double accelerationPath);
