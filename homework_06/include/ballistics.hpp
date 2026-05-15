#pragma once

struct BallisticsInput {
  double xd;
  double yd;
  double zd;
  double targetX;
  double targetY;
  double attackSpeed;
  double accelerationPath;
  char ammoName[32]{};
};

struct BallisticsResult {
  bool needManeuver;
  double maneuverX;
  double maneuverY;
  double fireX;
  double fireY;
};

void readFile(const char* filename, BallisticsInput& input);
void writeFile(const char* filename, const BallisticsResult& result);
bool calculateBallitsics(const BallisticsInput& input, BallisticsResult& result);