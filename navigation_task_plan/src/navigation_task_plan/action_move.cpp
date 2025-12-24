// Copyright 2025
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rosa_task_plan_plansys/rosa_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros_typedb_msgs/srv/query.hpp"

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
    // Declare fake_execution parameter
    this->declare_parameter("fake_execution", false);
    fake_execution_ = this->get_parameter("fake_execution").as_bool();
    
    // Declare fake execution duration (how long to simulate)
    this->declare_parameter("fake_execution_duration_ms", 1000);
    fake_duration_ms_ = this->get_parameter("fake_execution_duration_ms").as_int();
    this->declare_parameter("fake_time_per_meter", 1.0);
    fake_time_per_meter_ = this->get_parameter("fake_time_per_meter").as_double();
    
    if (fake_execution_) {
      RCLCPP_INFO(get_logger(), "🎭 Fake execution enabled");
    } else {
      RCLCPP_INFO(get_logger(), "🚀 Real execution enabled (using Nav2)");
    }
    
    // Create event publisher
    move_event_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/action_events", 10);
    
    // Create TypeDB client
    typedb_client_ = this->create_client<ros_typedb_msgs::srv::Query>(
      "/rosa_kb/query");
    
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
    this->declare_parameter("wp_11", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_12", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_13", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_14", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_15", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_16", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_17", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_18", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_19", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_20", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_21", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_22", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_23", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_24", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_25", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_26", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_27", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_28", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_29", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_30", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_31", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_32", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_33", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_34", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_35", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_36", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_37", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_38", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_39", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("wp_40", rclcpp::PARAMETER_DOUBLE_ARRAY);
    
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
  
  double getCorridorDistanceFromTypeDB(const std::string& from_wp, 
                                       const std::string& to_wp)
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query = 
      "match "
      "$c (from: $w1, to: $w2) isa corridor, has distance $d; "
      "$w1 has waypoint-name '" + from_wp + "'; "
      "$w2 has waypoint-name '" + to_wp + "'; "
      "fetch $d;";
    
    if (!typedb_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "TypeDB not available, using Euclidean distance");
      return -1.0;
    }
    
    auto future = typedb_client_->async_send_request(req);
    
    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      return -1.0;
    }
    
    auto response = future.get();
    
    if (!response->success || response->results.empty()) {
      return -1.0;
    }
    
    for (const auto &row : response->results) {
      for (const auto &attr : row.attributes) {
        if (attr.name == "d") {
          return attr.value.double_value;
        }
      }
    }
    
    return -1.0;
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
    fake_timer_fired_ = false;
    return rosa_task_plan_plansys::RosaAction::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    nav_goal_sent_ = false;
    if (fake_timer_) {
      fake_timer_->cancel();
      fake_timer_.reset();
      fake_timer_fired_ = false;
    }
    
    if (!fake_execution_ && navigate_cli_) {
      navigate_cli_->async_cancel_all_goals();
    }
    
    return rosa_task_plan_plansys::RosaAction::on_deactivate(previous_state);
  }

private:
  bool fake_execution_ = false;
  int fake_duration_ms_ = 1000;
  rclcpp::TimerBase::SharedPtr fake_timer_;
  std::string current_goal_wp_;
  bool fake_timer_fired_ = false;
  double fake_time_per_meter_ = 1.0;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_event_pub_;
  rclcpp::Client<ros_typedb_msgs::srv::Query>::SharedPtr typedb_client_;
  
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
    
    if (fake_execution_) {
      RCLCPP_INFO(get_logger(), "🎭 Fake execution enabled");

      auto args = get_arguments();
      std::string from_wp, to_wp, cfg;

      if (args[0] == "move_to_recharge") {
        if (args.size() >= 3) {
            from_wp = args[1];  
            to_wp = args[2];    
            cfg = "low_speed_config";
            
            RCLCPP_INFO(get_logger(), 
              "🔌 Recharge move: %s → %s", from_wp.c_str(), to_wp.c_str());
        } else {
            RCLCPP_ERROR(get_logger(),
              "move_to_recharge needs at least 3 args, got %zu", args.size());
            return;
        }
    }
      else if (args.size() >= 4 &&
              (args[0] == "move_lit" || args[0] == "move_dark")) {
          from_wp = args[1];
          to_wp   = args[2];
          cfg     = args[3];
      }
      else if (args.size() >= 3) {
          from_wp = args[0];
          to_wp   = args[1];
          cfg     = args[2];
      }
      else {
          RCLCPP_ERROR(get_logger(),
              "Cannot parse action arguments for fake execution (args=%zu)", args.size());
          return;
      }

      RCLCPP_INFO(get_logger(), "Fake move %s → %s (%s)",
                  from_wp.c_str(), to_wp.c_str(), cfg.c_str());

      // Try TypeDB first, fallback to Euclidean
      double dist = 4.0;
      

      // Get config speed
      double config_speed = 1.0;
      if (cfg.find("high_speed") != std::string::npos) {
        config_speed = 2.0;
      } else if (cfg.find("low_speed") != std::string::npos) {
        config_speed = 1.6;
      } else if (cfg.find("degraded") != std::string::npos) {
        config_speed = 1.2;
      }
      
      double time_seconds = dist / config_speed;
      int scaled_ms = static_cast<int>(time_seconds * 1000.0);
      
      RCLCPP_INFO(get_logger(), "Distance=%.2f m", dist);
      RCLCPP_INFO(get_logger(), "Config speed=%.2f m/s → duration=%.2f sec",
                  config_speed, time_seconds);
      
      // Publish START event
      auto event_msg = std_msgs::msg::String();
      event_msg.data = "MOVE_START|" + from_wp + "|" + to_wp + "|" + cfg + 
                       "|" + std::to_string(dist);
      move_event_pub_->publish(event_msg);
      
      fake_timer_fired_ = false;
      fake_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(scaled_ms),
      [this, from_wp, to_wp, cfg, dist]() {
        
        // ✅ Simple check: if timer already fired, ignore
        if (fake_timer_fired_) {
          return;
        }
        
        fake_timer_fired_ = true;
        
        // Publish END event
        auto end_msg = std_msgs::msg::String();
        end_msg.data = "MOVE_END|" + from_wp + "|" + to_wp + "|" + cfg +
                      "|" + std::to_string(dist);
        move_event_pub_->publish(end_msg);
        
        send_feedback(1.0, "Move completed (fake)");
        finish(true, 1.0, "Move completed (fake execution)");
        nav_goal_sent_ = false;
        
        // ✅ Cancel timer after execution
        if (fake_timer_) {
          fake_timer_->cancel();
          fake_timer_.reset();
        }
      });

      return;
    }
    
    // Real Nav2 execution
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
  auto node = std::make_shared<MoveAction>("action_move", 100ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}