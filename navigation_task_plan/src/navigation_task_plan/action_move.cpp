// Copyright 2025
// FIXED VERSION - No segfault on timer cleanup!
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rosa_task_plan_plansys/rosa_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

class MoveAction : public rosa_task_plan_plansys::RosaAction {
public:
  MoveAction(const std::string & node_name,
             const std::chrono::nanoseconds & rate)
    : rosa_task_plan_plansys::RosaAction(node_name, rate) {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    // ✅ Declare fake_execution parameter
    this->declare_parameter("fake_execution", false);
    fake_execution_ = this->get_parameter("fake_execution").as_bool();
    
    // ✅ Declare fake execution duration (how long to simulate)
    this->declare_parameter("fake_execution_duration_ms", 1000);
    fake_duration_ms_ = this->get_parameter("fake_execution_duration_ms").as_int();
    
    if (fake_execution_) {
      RCLCPP_INFO(get_logger(), "🎭 Fake execution enabled (duration: %dms)", fake_duration_ms_);
    } else {
      RCLCPP_INFO(get_logger(), "🚀 Real execution enabled (using Nav2)");
    }
    
    // Only create action client if NOT fake execution
    if (!fake_execution_) {
      callback_group_action_client_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

      navigate_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose",
        callback_group_action_client_);

      pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&MoveAction::current_pos_callback, this, _1));
    }

    // Declare waypoint parameters
    this->declare_parameter("wp_0", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_1", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_2", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_3", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_4", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_5", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_6", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_7", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_8", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_9", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_10", rclcpp::PARAMETER_DOUBLE_ARRAY);

    return plansys2::ActionExecutorClient::on_configure(previous_state);
  }

  void current_pos_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  double getDistance(const geometry_msgs::msg::Pose & pos1,
                     const geometry_msgs::msg::Pose & pos2)
  {
    return hypot(pos1.position.x - pos2.position.x,
                 pos1.position.y - pos2.position.y);
  }

  geometry_msgs::msg::PoseStamped get_waypoint(const std::string & waypoint)
  {
    auto vals = this->get_parameter(waypoint).as_double_array();

    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = now();
    wp.pose.position.x = vals[0];
    wp.pose.position.y = vals[1];
    wp.pose.position.z = vals[2];
    wp.pose.orientation.x = vals[3];
    wp.pose.orientation.y = vals[4];
    wp.pose.orientation.z = vals[5];
    wp.pose.orientation.w = vals[6];
    return wp;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    // Reset fake timer state
    fake_timer_fired_ = false;
    
    return rosa_task_plan_plansys::RosaAction::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    // ✅ FIXED: Safely cleanup timer
    if (fake_timer_) {
      fake_timer_->cancel();
      fake_timer_.reset();
      fake_timer_fired_ = false;
    }
    
    // Only cancel Nav2 goals if we have a real navigation client
    if (!fake_execution_ && navigate_cli_) {
      navigate_cli_->async_cancel_all_goals();
    }
    
    return rosa_task_plan_plansys::RosaAction::on_deactivate(previous_state);
  }

private:
  // ✅ Fake execution state
  bool fake_execution_ = false;
  int fake_duration_ms_ = 1000;
  rclcpp::TimerBase::SharedPtr fake_timer_;
  std::string current_goal_wp_;
  bool fake_timer_fired_ = false;  // ✅ FIXED: Prevent multiple firings
  
  // Real Nav2 state
  geometry_msgs::msg::Pose current_pos_;
  rclcpp::CallbackGroup::SharedPtr callback_group_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_cli_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  double dist_to_move_;
  bool nav_goal_sent_ = false;

  void do_work() override
  {
    if (!nav_goal_sent_) {
      send_nav_goal();
      nav_goal_sent_ = true;
    }
  }

  void send_nav_goal()
  {
    send_feedback(0.0, "Move starting");
    
    std::string goal_wp = get_arguments()[1];
    current_goal_wp_ = goal_wp;
    
    // ✅ FAKE EXECUTION PATH (ASYNCHRONOUS like Nav2!)
    if (fake_execution_) {
      RCLCPP_INFO(get_logger(), "🎭 Fake execution: Starting move to %s (will complete in %dms)", 
        goal_wp.c_str(), fake_duration_ms_);
      
      // Reset fired flag
      fake_timer_fired_ = false;
      
      // Create timer that fires repeatedly but only execute once
      fake_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(fake_duration_ms_),
        [this]() {
          // ✅ FIXED: Check if already fired (prevents double execution)
          if (fake_timer_fired_) {
            return;  // Already completed, ignore subsequent firings
          }
          fake_timer_fired_ = true;
          
          // This callback fires after the duration (like Nav2's result_callback)
          RCLCPP_INFO(get_logger(), "🎭 Fake execution: Completed move to %s", 
            current_goal_wp_.c_str());
          
          send_feedback(1.0, "Move completed (fake)");
          finish(true, 1.0, "Move completed (fake execution)");
          nav_goal_sent_ = false;
          
          // ✅ FIXED: DON'T cancel from inside callback!
          // Timer will be canceled in on_deactivate() instead
        }
      );
      
      // ✅ Return immediately - action is now RUNNING (just like Nav2!)
      return;
    }
    
    // ✅ REAL EXECUTION PATH (original Nav2 code)
    while (!navigate_cli_->wait_for_action_server(5s)) {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
    }
    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    nav2_msgs::action::NavigateToPose::Goal navigation_goal;
    navigation_goal.pose = get_waypoint(goal_wp);
    dist_to_move_ = getDistance(navigation_goal.pose.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0,
            1.0 - (feedback->distance_remaining / dist_to_move_))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
        nav_goal_sent_ = false;
      };

    future_navigation_goal_handle_ =
      navigate_cli_->async_send_goal(navigation_goal, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>("action_move", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}