#include <gtest/gtest.h>

#include "antidrone_turret/turret_logic.hpp"

namespace {

TEST(TurretLogicTest, LowConfidenceTargetStaysIdleAndSkipsTrigger)
{
  const auto target = antidrone_turret::TargetSample{true, 320.0F, 240.0F, 10.0F, 0.79F};

  const auto decision = antidrone_turret::evaluate_turret(
    target,
    antidrone_turret::ActuatorState::kReady);

  EXPECT_EQ(decision.target_state, antidrone_turret::TargetState::kLowConfidence);
  EXPECT_EQ(decision.action, antidrone_turret::TurretAction::kIdle);
  EXPECT_EQ(decision.trigger_state, antidrone_turret::TriggerDecision::kSkip);
  EXPECT_FALSE(decision.servo.has_value());
  EXPECT_FALSE(decision.gimbal.has_value());
}

TEST(TurretLogicTest, ServoDecisionMovesRightWhenTargetIsRightOfCenter)
{
  const auto target = antidrone_turret::TargetSample{true, 420.0F, 240.0F, 40.0F, 0.95F};

  const auto servo = antidrone_turret::make_servo_decision(target, 320.0F);

  EXPECT_EQ(servo.direction, antidrone_turret::ServoDirection::kRight);
  EXPECT_FLOAT_EQ(servo.target_x, 420.0F);
  EXPECT_FLOAT_EQ(servo.error_x, 100.0F);
}

TEST(TurretLogicTest, GimbalDecisionMovesUpWhenTargetIsAboveCenter)
{
  const auto target = antidrone_turret::TargetSample{true, 320.0F, 180.0F, 40.0F, 0.95F};

  const auto gimbal = antidrone_turret::make_gimbal_decision(target, 240.0F);

  EXPECT_EQ(gimbal.direction, antidrone_turret::GimbalDirection::kUp);
  EXPECT_FLOAT_EQ(gimbal.target_y, 180.0F);
  EXPECT_FLOAT_EQ(gimbal.error_y, 60.0F);
}

TEST(TurretLogicTest, CloseReadyTargetRequestsTrigger)
{
  const auto target = antidrone_turret::TargetSample{true, 320.0F, 240.0F, 25.0F, 0.90F};

  const auto decision = antidrone_turret::evaluate_turret(
    target,
    antidrone_turret::ActuatorState::kReady);

  EXPECT_EQ(decision.trigger_state, antidrone_turret::TriggerDecision::kRequested);
  EXPECT_TRUE(decision.should_trigger());
}

TEST(TurretLogicTest, CloseReloadingTargetMarksReloadingInsteadOfTrigger)
{
  const auto target = antidrone_turret::TargetSample{true, 320.0F, 240.0F, 25.0F, 0.90F};

  const auto decision = antidrone_turret::evaluate_turret(
    target,
    antidrone_turret::ActuatorState::kReloading);

  EXPECT_EQ(decision.trigger_state, antidrone_turret::TriggerDecision::kReloading);
  EXPECT_FALSE(decision.should_trigger());
}

TEST(TurretLogicTest, ValidFarTargetTracksButSkipsTrigger)
{
  const auto target = antidrone_turret::TargetSample{true, 340.0F, 230.0F, 55.0F, 0.82F};

  const auto decision = antidrone_turret::evaluate_turret(
    target,
    antidrone_turret::ActuatorState::kReady);

  EXPECT_EQ(decision.target_state, antidrone_turret::TargetState::kLocked);
  EXPECT_EQ(decision.action, antidrone_turret::TurretAction::kTrack);
  EXPECT_EQ(decision.trigger_state, antidrone_turret::TriggerDecision::kSkip);
  ASSERT_TRUE(decision.servo.has_value());
  ASSERT_TRUE(decision.gimbal.has_value());
}

}
