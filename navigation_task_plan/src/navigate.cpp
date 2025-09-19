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
#include <ctime>

// #include "std_msgs/msg/float32.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "lifecycle_msgs/msg/transition_event.hpp"

#include "navigate.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace navigation_task_plan
{

  NavigationController::NavigationController(const std::string &node_name)
      : rclcpp::Node(node_name)
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>("rosa_plansys_controller_executor");

    step_timer_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // TODO: create parameter for timer rate?albert_pddl
    step_timer_ = this->create_wall_timer(
        1s, std::bind(&NavigationController::step, this), step_timer_cb_group_);

    ros_typedb_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    typedb_query_cli_ = this->create_client<ros_typedb_msgs::srv::Query>(
        "ros_typedb/query",
        rmw_qos_profile_services_default,
        ros_typedb_cb_group_);
  }

  NavigationController::~NavigationController()
  {
  }

 void NavigationController::execute_plan()
  {
    this->fetch_waypoints();
    this->fetch_corridors();
    this->fetch_lighting();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value())
    {
      std::cout << "=== PLAN NOT FOUND ===" << std::endl;
      std::cout << "Current Goal: " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;

      std::cout << "--- Instances ---" << std::endl;
      for (const auto &instance : problem_expert_->getInstances())
      {
        std::cout << "Instance: " << instance.name << " of type: " << instance.type << std::endl;
      }

      std::cout << "--- Predicates ---" << std::endl;
      for (const auto &predicate : problem_expert_->getPredicates())
      {
        std::cout << parser::pddl::toString(predicate) << std::endl;
      }

      return;
    }

    std::cout << "=== PLAN FOUND ===" << std::endl;
    for (const auto &item : plan->items)
    {
      RCLCPP_INFO(this->get_logger(), "  Action: '%s'", item.action.c_str());
    }

    executor_client_->start_plan_execution(plan.value());
  }


  void NavigationController::finish_controlling()
  {
    this->step_timer_->cancel();
    this->executor_client_->cancel_plan_execution();
  }

  void NavigationController::step()
  {
    if (first_iteration_)
    {
      this->execute_plan();
      first_iteration_ = false;
      return;
    }

    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
    {
      if (executor_client_->getResult().value().success)
      {
        RCLCPP_INFO(this->get_logger(), "Plan execution finished with success!");
        this->finish_controlling();
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Replanning!");
        this->execute_plan();
        return;
      }
    }

    auto feedback = executor_client_->getFeedBack();
    for (const auto &action_feedback : feedback.action_execution_status)
    {
      if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
      {
        std::string error_str_ = "[" + action_feedback.action + "] finished with error: " + action_feedback.message_status;
        RCLCPP_ERROR(this->get_logger(), error_str_.c_str());
        break;
      }

      std::string arguments_str_ = " ";
      for (const auto &arguments : action_feedback.arguments)
      {
        arguments_str_ += arguments + " ";
      }
      std::string feedback_str_ = "[" + action_feedback.action + arguments_str_ +
                                  std::to_string(action_feedback.completion * 100.0) + "%]";
      RCLCPP_INFO(this->get_logger(), feedback_str_.c_str());
    }
  }

  // void NavigationController::fetch_items(){
  //   RCLCPP_INFO(this->get_logger(), " Fecthing items");
  //   if(!typedb_query_cli_->service_is_ready()){
  //     return;
  //   }

  //   auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  //   request->query_type = "fetch";
  //   request->query = "match $item isa item; fetch $item: item-name;";

  //   auto typedb_query_result_ = typedb_query_cli_->async_send_request(request);

  //   // Wait for the result.
  //   if (typedb_query_result_.wait_for(1s) != std::future_status::ready) {
  //     return;
  //   }

  //   auto result_ = typedb_query_result_.get();
  //   if(!result_->success){
  //     return;
  //   }

  //   for (const auto& result: result_->results) {
  //     for (const auto& attribute: result.attributes) {
  //       if (attribute.label == "item-name") {
  //         problem_expert_->addInstance(plansys2::Instance(attribute.value.string_value, "item"));
  //       }
  //     }
  //   }
  // }

  void NavigationController::fetch_waypoints()
{
  if (!typedb_query_cli_->service_is_ready()) return;

  auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  request->query_type = "fetch";
  request->query = "match $wp isa waypoint, has waypoint-name $name; fetch $name;";

  auto result = typedb_query_cli_->async_send_request(request);
  if (result.wait_for(1s) != std::future_status::ready) return;

  auto response = result.get();
  if (!response->success) return;

  for (const auto &res : response->results) {
    for (const auto &attr : res.attributes) {
      if (attr.label == "name") {
        problem_expert_->addInstance(plansys2::Instance(attr.value.string_value, "waypoint"));
      }
    }
  }
}

void NavigationController::fetch_corridors()
{
  if (!typedb_query_cli_->service_is_ready()) return;

  auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  request->query_type = "fetch";
  request->query =
    "match $c (from:$w1, to:$w2) isa corridor; "
    "$w1 has waypoint-name $w1_name; "
    "$w2 has waypoint-name $w2_name; "
    "fetch $w1_name; $w2_name;";

  auto result = typedb_query_cli_->async_send_request(request);
  if (result.wait_for(1s) != std::future_status::ready) return;

  auto response = result.get();
  if (!response->success) return;

  for (const auto &res : response->results) {
    std::string w1, w2;
    for (const auto &attr : res.attributes) {
      if (attr.label == "w1_name") w1 = attr.value.string_value;
      else if (attr.label == "w2_name") w2 = attr.value.string_value;
    }
    problem_expert_->addPredicate(plansys2::Predicate("(path " + w1 + " " + w2 + ")"));
  }
}

void NavigationController::fetch_lighting()
{
  if (!typedb_query_cli_->service_is_ready()) return;

  auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  request->query_type = "fetch";
  request->query =
    "match $c (from:$w1, to:$w2) isa corridor; "
    "$w1 has waypoint-name $w1_name; "
    "$w2 has waypoint-name $w2_name; "
    "$l (from:$w1, to:$w2) isa lighting-condition, has is-dark true; "
    "fetch $w1_name; $w2_name;";

  auto result = typedb_query_cli_->async_send_request(request);
  if (result.wait_for(1s) != std::future_status::ready) return;

  auto response = result.get();
  if (!response->success) return;

  for (const auto &res : response->results) {
    std::string w1, w2;
    for (const auto &attr : res.attributes) {
      if (attr.label == "w1_name") w1 = attr.value.string_value;
      else if (attr.label == "w2_name") w2 = attr.value.string_value;
    }
    problem_expert_->addPredicate(plansys2::Predicate("(is-dark-" + w1 + "-" + w2 + ")"));
  }
}


  // void NavigationController::fetch_delivery_locations() {
  //   if(!typedb_query_cli_->service_is_ready()){
  //     return;
  //   }

  //   auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  //   request->query_type = "fetch";
  //   request->query = "match $item isa item, has item-name $item-name; "
  //     "(item:$item, room:$room) isa delivery-location;"
  //     "$room has room-name $room-name;"
  //     "fetch $item-name; $room-name;";

  //   auto typedb_query_result_ = typedb_query_cli_->async_send_request(request);

  //   // Wait for the result.
  //   if (typedb_query_result_.wait_for(1s) != std::future_status::ready) {
  //     return;
  //   }

  //   auto result_ = typedb_query_result_.get();
  //   if(!result_->success){
  //     return;
  //   }

  //   for (const auto& result: result_->results) {
  //     std::string item_name;
  //     std::string room_name;
  //     for (const auto& attribute: result.attributes) {
  //       if (attribute.label == "item-name") {
  //         item_name = attribute.value.string_value;
  //       } else if (attribute.label == "room-name") {
  //         room_name = attribute.value.string_value;
  //       }
  //     }
  //     auto predicate = "(item_delivery_location " + item_name + " " + room_name + ")";
  //     problem_expert_->addPredicate(plansys2::Predicate(predicate));
  //   }
  // }

} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_task_plan::NavigationController>(
      "navigate_controller");

  rclcpp::Rate rate(5);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
}
