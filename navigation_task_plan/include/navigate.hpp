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

// Needed for parser::pddl::fromString()
#include "plansys2_pddl_parser/Utils.hpp"

#include "rosa_task_plan_plansys/rosa_plansys_controller.hpp"

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

  // --- Core logic ---
  void build_problem_from_kb();  // NEW: dynamically constructs the problem from TypeDB
  void execute_plan();
  void step();
  void finish_controlling();

  // --- (Legacy fetch functions no longer used but kept for reference/testing) ---
  void fetch_waypoints();
  void fetch_corridors();
  void fetch_lighting();
};

}  // namespace navigation_task_plan

#endif  // NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_
