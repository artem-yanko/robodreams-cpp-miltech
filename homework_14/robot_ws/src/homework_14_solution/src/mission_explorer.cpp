#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/scenario.hpp"
#include "underground_world/state_qos.hpp"
#include "underground_world/srv/payload_trigger.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using underground_world::Position;
using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::RobotResult;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

constexpr auto kLocalScanTopic = "/robot/local_scan";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kResultTopic = "/robot/result";
constexpr auto kStatusTopic = "/student/status";
constexpr auto kPayloadTriggerService = "/payload/trigger";

constexpr char kUnknownCell = '?';
constexpr char kWallCell = '#';

std::uint8_t direction_from_step(const Position from, const Position to)
{
  const Position delta{to.x - from.x, to.y - from.y};
  if (delta.x == 0 && delta.y == -1) {
    return MoveCommand::UP;
  }
  if (delta.x == 0 && delta.y == 1) {
    return MoveCommand::DOWN;
  }
  if (delta.x == -1 && delta.y == 0) {
    return MoveCommand::LEFT;
  }
  return MoveCommand::RIGHT;
}

struct ContactInfo {
  int id = 0;
  Position position;
};

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
  : Node("mission_explorer")
  {
    local_scan_sub_ = create_subscription<LocalScan>(
      kLocalScanTopic,
      underground_world::make_state_qos(),
      [this](const LocalScan::SharedPtr msg) { on_local_scan(*msg); });

    result_sub_ = create_subscription<RobotResult>(
      kResultTopic,
      underground_world::make_state_qos(),
      [this](const RobotResult::SharedPtr msg) { latest_result_ = *msg; });

    move_pub_ = create_publisher<MoveCommand>(kMoveTopic, rclcpp::QoS{10});
    status_pub_ = create_publisher<StudentStatus>(kStatusTopic, rclcpp::QoS{10});
    payload_trigger_client_ = create_client<PayloadTrigger>(kPayloadTriggerService);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds{20},
      [this]() { control_tick(); });
  }

private:
  void on_local_scan(const LocalScan & scan)
  {
    have_scan_ = true;
    ++scan_sequence_;

    current_position_ = Position{scan.robot_x, scan.robot_y};

    visible_contacts_.clear();

    for (const auto & cell : scan.cells) {
      const Position position{cell.x, cell.y};
      known_cells_[position] = cell.cell_type.empty() ? kUnknownCell : cell.cell_type.front();

      if (cell.cell_type == "C") {
        active_contacts_[cell.contact_id] = position;
        visible_contacts_.push_back(ContactInfo{cell.contact_id, position});
      } else if (cell.cell_type == "x") {
        active_contacts_.erase(cell.contact_id);
        processed_contacts_.insert(cell.contact_id);
      }
    }

    for (auto it = active_contacts_.begin(); it != active_contacts_.end();) {
      if (processed_contacts_.find(it->first) != processed_contacts_.end()) {
        it = active_contacts_.erase(it);
      } else {
        ++it;
      }
    }

    if (pending_contact_id_.has_value()) {
      const bool still_visible = std::any_of(
        visible_contacts_.begin(),
        visible_contacts_.end(),
        [this](const ContactInfo & contact) { return contact.id == pending_contact_id_.value(); });
      if (!still_visible) {
        pending_contact_id_.reset();
      }
    }
  }

  void control_tick()
  {
    if (!have_scan_ || acted_scan_sequence_ == scan_sequence_) {
      return;
    }

    bool action_sent = false;

    if (latest_result_.mission_result == "SUCCESS") {
      publish_status(StudentStatus::DONE);
      action_sent = true;
    } else if (latest_result_.mission_result == "FAILED_MAX_STEPS") {
      publish_status(StudentStatus::FAILED);
      action_sent = true;
    } else if (pending_contact_id_.has_value()) {
      publish_status(StudentStatus::ENGAGING);
      action_sent = true;
    } else if (!visible_contacts_.empty()) {
      action_sent = engage_contact(visible_contacts_.front());
    } else {
      const auto next_step = plan_next_step();
      if (!next_step.has_value()) {
        publish_status(StudentStatus::DONE);
        action_sent = true;
      } else {
        MoveCommand command;
        command.direction = direction_from_step(current_position_, next_step.value());
        move_pub_->publish(command);
        publish_status(StudentStatus::EXPLORING);
        action_sent = true;
      }
    }

    if (action_sent) {
      acted_scan_sequence_ = scan_sequence_;
    }
  }

  bool engage_contact(const ContactInfo & contact)
  {
    publish_status(StudentStatus::ENGAGING);
    if (!payload_trigger_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "waiting for %s service",
        kPayloadTriggerService);
      return false;
    }

    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = contact.id;
    request->x = contact.position.x;
    request->y = contact.position.y;

    pending_contact_id_ = contact.id;
    payload_trigger_client_->async_send_request(
      request,
      [this](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
        const auto response = future.get();
        if (!response->accepted) {
          RCLCPP_WARN(
            get_logger(),
            "payload trigger rejected: %s",
            response->reason.c_str());
          pending_contact_id_.reset();
        }
      });
    return true;
  }

  std::optional<Position> plan_next_step() const
  {
    std::queue<Position> pending;
    std::map<Position, Position> parent;
    std::set<Position> visited;

    pending.push(current_position_);
    visited.insert(current_position_);

    while (!pending.empty()) {
      const Position current = pending.front();
      pending.pop();

      if (is_frontier(current)) {
        return reconstruct_first_step(current, parent);
      }

      for (const auto & neighbor : cardinal_neighbors(current)) {
        if (visited.find(neighbor) != visited.end() || !is_traversable(neighbor)) {
          continue;
        }

        visited.insert(neighbor);
        parent.emplace(neighbor, current);
        pending.push(neighbor);
      }
    }

    return std::nullopt;
  }

  std::optional<Position> reconstruct_first_step(
    const Position goal,
    const std::map<Position, Position> & parent) const
  {
    if (goal == current_position_) {
      for (const auto & neighbor : cardinal_neighbors(current_position_)) {
        if (is_traversable(neighbor) && is_frontier(neighbor)) {
          return neighbor;
        }
      }
      return std::nullopt;
    }

    Position step = goal;
    auto it = parent.find(step);
    while (it != parent.end() && !(it->second == current_position_)) {
      step = it->second;
      it = parent.find(step);
    }

    if (it == parent.end()) {
      return std::nullopt;
    }

    return step;
  }

  bool is_frontier(const Position position) const
  {
    if (!is_traversable(position)) {
      return false;
    }

    for (const auto & neighbor : cardinal_neighbors(position)) {
      if (known_cells_.find(neighbor) == known_cells_.end()) {
        return true;
      }
    }

    return false;
  }

  bool is_traversable(const Position position) const
  {
    const auto it = known_cells_.find(position);
    if (it == known_cells_.end()) {
      return false;
    }

    if (it->second == kWallCell || it->second == kUnknownCell) {
      return false;
    }

    for (const auto & [contact_id, contact_position] : active_contacts_) {
      if (processed_contacts_.find(contact_id) == processed_contacts_.end() && contact_position == position) {
        return false;
      }
    }

    return true;
  }

  std::vector<Position> cardinal_neighbors(const Position position) const
  {
    return {
      Position{position.x, position.y - 1},
      Position{position.x, position.y + 1},
      Position{position.x - 1, position.y},
      Position{position.x + 1, position.y},
    };
  }

  void publish_status(const std::uint8_t state) const
  {
    StudentStatus status;
    status.state = state;
    status_pub_->publish(status);
  }

  bool have_scan_ = false;
  std::uint64_t scan_sequence_ = 0;
  std::uint64_t acted_scan_sequence_ = 0;
  Position current_position_{};
  std::optional<int> pending_contact_id_;
  RobotResult latest_result_{};
  std::map<Position, char> known_cells_;
  std::map<int, Position> active_contacts_;
  std::set<int> processed_contacts_;
  std::vector<ContactInfo> visible_contacts_;
  rclcpp::Subscription<LocalScan>::SharedPtr local_scan_sub_;
  rclcpp::Subscription<RobotResult>::SharedPtr result_sub_;
  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr payload_trigger_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
