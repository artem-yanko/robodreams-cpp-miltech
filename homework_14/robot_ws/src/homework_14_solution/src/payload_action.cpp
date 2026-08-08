#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {

constexpr auto kEnemyDownTopic = "/payload/enemy_down";
constexpr auto kPayloadTriggerService = "/payload/trigger";

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
  : Node("payload_action")
  {
    enemy_down_pub_ = create_publisher<EnemyDown>(kEnemyDownTopic, rclcpp::QoS{10});
    trigger_service_ = create_service<PayloadTrigger>(
      kPayloadTriggerService,
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<PayloadTrigger::Request> request,
        std::shared_ptr<PayloadTrigger::Response> response) {
        EnemyDown msg;
        msg.contact_id = request->contact_id;
        msg.x = request->x;
        msg.y = request->y;

        enemy_down_pub_->publish(msg);

        response->accepted = true;
        response->reason = "enemy_down published";

        RCLCPP_INFO(
          get_logger(),
          "triggered contact_id=%d at (%d,%d)",
          request->contact_id,
          request->x,
          request->y);
      });
  }

private:
  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_pub_;
  rclcpp::Service<PayloadTrigger>::SharedPtr trigger_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
