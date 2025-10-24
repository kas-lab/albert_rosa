// Copyright 2024
#ifndef NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_
#define NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <tuple>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros_typedb_msgs/srv/query.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "rosa_task_plan_plansys/rosa_plansys_controller.hpp"
#include "rosa_msgs/srv/selected_configurations.hpp"
#include "rosa_msgs/msg/component_configuration.hpp"

namespace navigation_task_plan
{

class NavigationController : public rclcpp::Node
{
public:
  explicit NavigationController(const std::string & node_name);
  ~NavigationController() override;

private:
  // --- KB helpers ---
  double getBatteryFromKB();
  std::vector<std::string> getFeasibleConfigsFromKB();

  // Current applied configuration (updated on successful adaptation)
  std::string current_config_ = "high_speed_config";

protected:
  // --- ROS callback groups, timers, clients ---
  rclcpp::CallbackGroup::SharedPtr step_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  rclcpp::CallbackGroup::SharedPtr ros_typedb_cb_group_;
  rclcpp::Client<ros_typedb_msgs::srv::Query>::SharedPtr typedb_query_cli_;

  // PlanSys2 clients
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  // Execution state
  bool first_iteration_ = true;
  std::vector<std::pair<std::string, std::string>> corridor_pairs_;

  // --- Proactive state & timers ---
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr proactive_timer_;   // reactive feasibility check
  rclcpp::TimerBase::SharedPtr future_timer_;      // proactive prediction + replan

  double battery_level_ = 100.0;   // %
  double predicted_cost_ = 0.0;    // %
  double safety_margin_ = 10.0;    // % (optional, if you decide to use it)
  struct ParsedAction {
  std::string action_name;
  std::string from;
  std::string to;
  std::string config;
  double cost;
  };

  std::vector<ParsedAction> current_plan_actions_;
  // === Core control loop ===
  void step();
  void execute_plan();
  void finish_controlling();
    // Proactive helpers
  std::tuple<std::string, std::string, std::string> parse_action(const std::string &action);
  std::map<std::tuple<std::string, std::string, std::string>, double> cost_map_;

  // Build PDDL problem from KB
  void build_problem_from_kb();

  // Fetchers (KB → PDDL problem)
  void fetch_waypoints();
  void fetch_corridors();
  void fetch_configurations();
  // void fetch_config_traits();
  void fetch_lighting_conditions();
  void fetch_energy_costs();
  void fetch_battery_and_feasibility();  // KB-driven feasibility to PDDL
  void fetch_goal();

  // Monitoring & adaptation
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // Reactive layer (now)
  void evaluatePlanFeasibility();

  // Proactive layer (future prediction + replan)
  void evaluateFutureFeasibility();
  void triggerReplan();

  // Architectural adaptation helper
  void triggerProactiveAdaptation(const std::string & target_cfg);

  // (Optional) simple example function kept for reference
  double computePredictedCost();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rosa_event_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rosa_reconfig_sub_;
  bool adaptation_triggered_ = false;
};

}  // namespace navigation_task_plan

#endif  // NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_
