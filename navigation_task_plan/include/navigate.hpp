// Copyright 2024 Gustavo Rezende Silva
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_
#define NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ros_typedb_msgs/srv/query.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "rosa_task_plan_plansys/rosa_plansys_controller.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
using namespace std::chrono_literals;



namespace navigation_task_plan
{

class NavigationController : public rclcpp::Node
{
public:
  explicit NavigationController(const std::string & node_name);
  virtual ~NavigationController();

protected:
  // --- ROS callback groups and timers ---
  rclcpp::CallbackGroup::SharedPtr step_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  rclcpp::CallbackGroup::SharedPtr ros_typedb_cb_group_;
  rclcpp::Client<ros_typedb_msgs::srv::Query>::SharedPtr typedb_query_cli_;

  // --- PlanSys2 core clients ---
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  // --- Execution state ---
  bool first_iteration_ = true;
  std::vector<std::pair<std::string, std::string>> corridor_pairs_; 
  // --- Core logic ---
  void build_problem_from_kb();  // NEW: dynamically constructs the problem from TypeDB
  void execute_plan();
  void step();
  void finish_controlling();

  void fetch_waypoints();
  void fetch_corridors();
  // --- Proactive Adaptation ---
  double battery_level_ = 100.0;
  double predicted_cost_ = 0.0;
  double safety_margin_ = 10.0;

  rclcpp::TimerBase::SharedPtr proactive_timer_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  void fetch_configurations();
  void fetch_config_traits();
  void fetch_lighting_conditions();
  void fetch_energy_costs();
  void fetch_battery_and_feasibility();
  void fetch_goal();
  // === Proactive adaptation functions ===
  void start_proactive_monitoring();
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void evaluatePlanFeasibility();
  void triggerProactiveAdaptation();
  double computePredictedCost();
  double get_predicted_cost_from_kb();
  double get_battery_level_from_kb();

};

}  // namespace navigation_task_plan

#endif  // NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_
