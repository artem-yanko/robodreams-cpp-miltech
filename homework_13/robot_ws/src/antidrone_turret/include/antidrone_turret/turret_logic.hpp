#pragma once

#include <cstdint>
#include <optional>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/target_sequence.hpp"

namespace antidrone_turret {

enum class TargetState : std::uint8_t {
  kNone = 0,
  kLowConfidence = 1,
  kLocked = 2,
};

enum class TurretAction : std::uint8_t {
  kIdle = 0,
  kTrack = 1,
};

enum class TriggerDecision : std::uint8_t {
  kSkip = 0,
  kRequested = 1,
  kReloading = 2,
};

enum class ServoDirection : std::int8_t {
  kLeft = -1,
  kCenter = 0,
  kRight = 1,
};

enum class GimbalDirection : std::int8_t {
  kDown = -1,
  kCenter = 0,
  kUp = 1,
};

struct ServoDecision {
  ServoDirection direction{ServoDirection::kCenter};
  float target_x{0.0F};
  float error_x{0.0F};
};

struct GimbalDecision {
  GimbalDirection direction{GimbalDirection::kCenter};
  float target_y{0.0F};
  float error_y{0.0F};
};

struct TurretControllerConfig {
  float confidence_threshold{0.80F};
  float max_distance_m{30.0F};
  float frame_center_x{320.0F};
  float frame_center_y{240.0F};
};

struct TurretDecision {
  TargetState target_state{TargetState::kNone};
  TurretAction action{TurretAction::kIdle};
  TriggerDecision trigger_state{TriggerDecision::kSkip};
  float confidence{0.0F};
  float distance_m{0.0F};
  std::optional<ServoDecision> servo;
  std::optional<GimbalDecision> gimbal;

  [[nodiscard]] bool should_trigger() const
  {
    return trigger_state == TriggerDecision::kRequested;
  }
};

inline TargetState evaluate_target(
  const TargetSample& target,
  const float confidence_threshold)
{
  if (!target.visible) {
    return TargetState::kNone;
  }

  if (target.confidence < confidence_threshold) {
    return TargetState::kLowConfidence;
  }

  return TargetState::kLocked;
}

inline ServoDecision make_servo_decision(
  const TargetSample& target,
  const float frame_center_x)
{
  const float error_x = target.x - frame_center_x;

  ServoDirection direction = ServoDirection::kCenter;
  if (error_x > 0.0F) {
    direction = ServoDirection::kRight;
  } else if (error_x < 0.0F) {
    direction = ServoDirection::kLeft;
  }

  return ServoDecision{direction, target.x, error_x};
}

inline GimbalDecision make_gimbal_decision(
  const TargetSample& target,
  const float frame_center_y)
{
  const float error_y = frame_center_y - target.y;

  GimbalDirection direction = GimbalDirection::kCenter;
  if (error_y > 0.0F) {
    direction = GimbalDirection::kUp;
  } else if (error_y < 0.0F) {
    direction = GimbalDirection::kDown;
  }

  return GimbalDecision{direction, target.y, error_y};
}

inline TriggerDecision decide_trigger(
  const TargetSample& target,
  const float max_distance_m,
  const ActuatorState actuator_state)
{
  if (target.distance_m > max_distance_m) {
    return TriggerDecision::kSkip;
  }

  if (actuator_state == ActuatorState::kReloading) {
    return TriggerDecision::kReloading;
  }

  return TriggerDecision::kRequested;
}

inline TurretDecision evaluate_turret(
  const TargetSample& target,
  const ActuatorState actuator_state,
  const TurretControllerConfig& config = {})
{
  TurretDecision decision{};
  decision.confidence = target.confidence;
  decision.distance_m = target.distance_m;
  decision.target_state = evaluate_target(target, config.confidence_threshold);

  if (decision.target_state != TargetState::kLocked) {
    return decision;
  }

  decision.action = TurretAction::kTrack;
  decision.servo = make_servo_decision(target, config.frame_center_x);
  decision.gimbal = make_gimbal_decision(target, config.frame_center_y);
  decision.trigger_state = decide_trigger(target, config.max_distance_m, actuator_state);
  return decision;
}

}  // namespace antidrone_turret
