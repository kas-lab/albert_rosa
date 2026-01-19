#include "std_msgs/msg/bool.hpp"
#include "rosa_task_plan_plansys/rosa_action.hpp"

using namespace std::chrono_literals;

class RechargeAction : public rosa_task_plan_plansys::RosaAction
{
public:
  RechargeAction(const std::string & node_name,
                 const std::chrono::nanoseconds & rate)
  : RosaAction(node_name, rate)
  {
    // Publisher: to trigger charging in the battery monitor
    recharge_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/battery_monitor/recharge", 10);

    // Subscriber: to detect when charging is complete
    recharge_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/battery_monitor/recharge_complete",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          RCLCPP_INFO(get_logger(), "✅ Recharge complete signal received!");
          recharged_ = true;
        }
      });
  }
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    recharged_ = false;
    publish_recharge_command(true);
    RCLCPP_INFO(get_logger(), "⚡ Starting recharge...");
    return RosaAction::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    publish_recharge_command(false);
    RCLCPP_INFO(get_logger(), "🔌 Recharge action deactivated.");
    return RosaAction::on_deactivate(previous_state);
  }

private:
  bool recharged_ = false;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recharge_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recharge_complete_sub_;
  
  void publish_recharge_command(bool state)
  {
    std_msgs::msg::Bool msg;
    msg.data = state;
    recharge_pub_->publish(msg);
  }

  void do_work() override
  {
    if (recharged_) {
      publish_recharge_command(false);  // stop charging
      RCLCPP_INFO(get_logger(), "✅ Battery fully recharged. Ending action.");
      finish(true, 1.0, "Recharge complete");
    } else {
      RCLCPP_DEBUG(get_logger(), "Charging in progress...");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RechargeAction>("recharge", 500ms);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
