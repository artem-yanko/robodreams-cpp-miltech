#include <gtest/gtest.h>
#include "ballistics.hpp"

// Створення тестового набору
TEST(calculateBallitsicsTest, BallisticsInputTest)
{
  BallisticsInput input{};
  input.xd = 0.0;
  input.yd = 0.0;
  input.zd = 15.0;
  input.targetX = 20.0;
  input.targetY = 10.0;
  input.attackSpeed = 10.0;
  input.accelerationPath = 25.0;
  strcpy(input.ammoName, "VOG-17");

  BallisticsResult result{};

  EXPECT_TRUE(calculateBallitsics(input, result));
  EXPECT_TRUE(result.needManeuver);
  EXPECT_NEAR(result.maneuverX, -16.3428, 1e-3);
  EXPECT_NEAR(result.maneuverY, -8.17142, 1e-3);
  EXPECT_NEAR(result.fireX, 6.01785, 1e-3);
  EXPECT_NEAR(result.fireY, 3.00892, 1e-3);
}

TEST(calculateBallitsicsTest, BallisticsUnknownAmmoTest)
{
  BallisticsInput input{};
  input.xd = 0.0;
  input.yd = 0.0;
  input.zd = 15.0;
  input.targetX = 20.0;
  input.targetY = 10.0;
  input.attackSpeed = 10.0;
  input.accelerationPath = 25.0;
  strcpy(input.ammoName, "UNKNOWN");

  BallisticsResult result{};

  EXPECT_FALSE(calculateBallitsics(input, result));
}
