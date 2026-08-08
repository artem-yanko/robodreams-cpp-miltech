#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/turret_logic.hpp"

namespace {

constexpr auto kTargetTopic = "/perception/target";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kGimbalCommandTopic = "/gimbal/cmd";
constexpr auto kServoCommandTopic = "/servo/cmd";
constexpr auto kTurretStatusTopic = "/turret/status";
constexpr auto kTriggerService = "/actuator/trigger";

std::uint8_t to_message_target_state(const antidrone_turret::TargetState state)
{
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  switch (state) {
    case antidrone_turret::TargetState::kNone:
      return TurretStatus::TARGET_NONE;
    case antidrone_turret::TargetState::kLowConfidence:
      return TurretStatus::TARGET_LOW_CONFIDENCE;
    case antidrone_turret::TargetState::kLocked:
      return TurretStatus::TARGET_LOCKED;
  }

  return TurretStatus::TARGET_NONE;
}

std::uint8_t to_message_action(const antidrone_turret::TurretAction action)
{
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  return action == antidrone_turret::TurretAction::kTrack
           ? TurretStatus::ACTION_TRACK
           : TurretStatus::ACTION_IDLE;
}

std::uint8_t to_message_trigger_state(const antidrone_turret::TriggerDecision trigger_state)
{
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  switch (trigger_state) {
    case antidrone_turret::TriggerDecision::kSkip:
      return TurretStatus::TRIGGER_SKIP;
    case antidrone_turret::TriggerDecision::kRequested:
      return TurretStatus::TRIGGER_REQUESTED;
    case antidrone_turret::TriggerDecision::kReloading:
      return TurretStatus::TRIGGER_RELOADING;
  }

  return TurretStatus::TRIGGER_SKIP;
}

std::int8_t to_message_servo_direction(const antidrone_turret::ServoDirection direction)
{
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  switch (direction) {
    case antidrone_turret::ServoDirection::kLeft:
      return ServoCommand::LEFT;
    case antidrone_turret::ServoDirection::kCenter:
      return ServoCommand::CENTER;
    case antidrone_turret::ServoDirection::kRight:
      return ServoCommand::RIGHT;
  }

  return ServoCommand::CENTER;
}

std::int8_t to_message_gimbal_direction(const antidrone_turret::GimbalDirection direction)
{
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  switch (direction) {
    case antidrone_turret::GimbalDirection::kDown:
      return GimbalCommand::DOWN;
    case antidrone_turret::GimbalDirection::kCenter:
      return GimbalCommand::CENTER;
    case antidrone_turret::GimbalDirection::kUp:
      return GimbalCommand::UP;
  }

  return GimbalCommand::CENTER;
}

antidrone_turret::ActuatorState from_message_state(const std::uint8_t state)
{
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  return state == ActuatorStatus::RELOADING
           ? antidrone_turret::ActuatorState::kReloading
           : antidrone_turret::ActuatorState::kReady;
}

antidrone_turret::TargetSample to_target_sample(const antidrone_turret::msg::Target& message)
{
  return antidrone_turret::TargetSample{
    message.visible,
    message.x,
    message.y,
    message.distance_m,
    message.confidence,
  };
}

antidrone_turret::msg::ServoCommand to_servo_message(
  const antidrone_turret::ServoDecision& decision)
{
  auto message = antidrone_turret::msg::ServoCommand{};
  message.direction = to_message_servo_direction(decision.direction);
  message.target_x = decision.target_x;
  message.error_x = decision.error_x;
  return message;
}

antidrone_turret::msg::GimbalCommand to_gimbal_message(
  const antidrone_turret::GimbalDecision& decision)
{
  auto message = antidrone_turret::msg::GimbalCommand{};
  message.direction = to_message_gimbal_direction(decision.direction);
  message.target_y = decision.target_y;
  message.error_y = decision.error_y;
  return message;
}

antidrone_turret::msg::TurretStatus to_status_message(
  const antidrone_turret::TurretDecision& decision)
{
  auto message = antidrone_turret::msg::TurretStatus{};
  message.target_state = to_message_target_state(decision.target_state);
  message.action = to_message_action(decision.action);
  message.trigger_state = to_message_trigger_state(decision.trigger_state);
  message.confidence = decision.confidence;
  message.distance_m = decision.distance_m;
  return message;
}

}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  using Target = antidrone_turret::msg::Target;
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

  TurretControllerNode()
    : Node("turret_controller_node")
  {
    config_.confidence_threshold =
      declare_parameter<double>("confidence_threshold", config_.confidence_threshold);
    config_.max_distance_m =
      declare_parameter<double>("max_distance_m", config_.max_distance_m);

    target_subscription_ = create_subscription<Target>(
      kTargetTopic,
      10,
      [this](const Target& target) { on_target(target); });

    actuator_status_subscription_ = create_subscription<ActuatorStatus>(
      kActuatorStatusTopic,
      10,
      [this](const ActuatorStatus& status) { on_actuator_status(status); });

    gimbal_publisher_ = create_publisher<GimbalCommand>(kGimbalCommandTopic, 10);
    servo_publisher_ = create_publisher<ServoCommand>(kServoCommandTopic, 10);
    turret_status_publisher_ = create_publisher<TurretStatus>(kTurretStatusTopic, 10);
    trigger_client_ = create_client<TriggerActuator>(kTriggerService);

    RCLCPP_INFO(
      get_logger(),
      "subscribed to %s and %s, publishing %s, %s, %s",
      kTargetTopic,
      kActuatorStatusTopic,
      kGimbalCommandTopic,
      kServoCommandTopic,
      kTurretStatusTopic);
  }

private:
  void on_target(const Target& target)
  {
    const auto decision = antidrone_turret::evaluate_turret(
      to_target_sample(target),
      actuator_state_,
      config_);

    turret_status_publisher_->publish(to_status_message(decision));

    if (decision.action == antidrone_turret::TurretAction::kTrack) {
      gimbal_publisher_->publish(to_gimbal_message(*decision.gimbal));
      servo_publisher_->publish(to_servo_message(*decision.servo));
    }

    if (!decision.should_trigger()) {
      return;
    }

    if (!trigger_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "trigger service is not ready");
      return;
    }

    auto request = std::make_shared<TriggerActuator::Request>();
    request->confidence = target.confidence;
    request->distance_m = target.distance_m;

    actuator_state_ = antidrone_turret::ActuatorState::kReloading;
    trigger_client_->async_send_request(
      request,
      [this](rclcpp::Client<TriggerActuator>::SharedFuture response_future) {
        const auto response = response_future.get();
        if (!response->accepted) {
          RCLCPP_WARN(get_logger(), "trigger request rejected by actuator");
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "trigger accepted, trigger_count=%u",
          response->trigger_count);
      });
  }

  void on_actuator_status(const ActuatorStatus& status)
  {
    actuator_state_ = from_message_state(status.state);
  }

  antidrone_turret::TurretControllerConfig config_{};
  antidrone_turret::ActuatorState actuator_state_{antidrone_turret::ActuatorState::kReady};
  rclcpp::Subscription<Target>::SharedPtr target_subscription_;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuator_status_subscription_;
  rclcpp::Publisher<GimbalCommand>::SharedPtr gimbal_publisher_;
  rclcpp::Publisher<ServoCommand>::SharedPtr servo_publisher_;
  rclcpp::Publisher<TurretStatus>::SharedPtr turret_status_publisher_;
  rclcpp::Client<TriggerActuator>::SharedPtr trigger_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}
