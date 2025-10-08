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
#include "plansys2_pddl_parser/Utils.hpp"

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
    rclcpp::CallbackGroupType::Reentrant); 
    typedb_query_cli_ = this->create_client<ros_typedb_msgs::srv::Query>(
        "ros_typedb/query",
        rclcpp::QoS(rclcpp::ServicesQoS()),
        ros_typedb_cb_group_);
  }

  NavigationController::~NavigationController()
  {
  }


 void NavigationController::build_problem_from_kb()
{
  problem_expert_->clearKnowledge();

  // ===== WAYPOINTS =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query = "match $wp isa waypoint, has waypoint-name $name; fetch $name;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Waypoint query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          for (const auto &attr : res.attributes) {
            if (attr.label == "name") {
              problem_expert_->addInstance(plansys2::Instance(attr.value.string_value, "waypoint"));
            }
          }
        }
      }
    }
  }

  // ===== CONFIGURATIONS =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query = "match $c isa configuration, has config-name $n; fetch $n;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Configuration query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          for (const auto &attr : res.attributes) {
            if (attr.label == "n") {
              auto cfg = attr.value.string_value;
              problem_expert_->addInstance(plansys2::Instance(cfg, "configuration"));
              problem_expert_->addPredicate(plansys2::Predicate("(can-use " + cfg + ")"));
              problem_expert_->addPredicate(plansys2::Predicate("(config-valid " + cfg + ")"));
            }
          }
        }
      }
    }
  }

  // ===== INITIAL POSITION =====
  problem_expert_->addPredicate(plansys2::Predicate("(at wp_0)"));

  // ===== CORRIDORS =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query =
      "match $c (from:$w1, to:$w2) isa corridor; "
      "$w1 has waypoint-name $w1n; $w2 has waypoint-name $w2n; "
      "fetch $w1n; $w2n;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Corridor query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          std::string a, b;
          for (const auto &attr : res.attributes) {
            if (attr.label == "w1n") a = attr.value.string_value;
            if (attr.label == "w2n") b = attr.value.string_value;
          }
          if (!a.empty() && !b.empty()) {
            problem_expert_->addPredicate(plansys2::Predicate("(is-corridor " + a + " " + b + ")"));
          }
        }
      }
    }
  }

  // ===== LIGHTING =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query =
      "match $l (from:$w1, to:$w2) isa lighting-condition, has is-dark $isd, has is-lit $isl; "
      "$w1 has waypoint-name $w1n; $w2 has waypoint-name $w2n; "
      "fetch $w1n; $w2n; $isd; $isl;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Lighting query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          std::string a, b;
          bool is_dark = false, is_lit = false;
          for (const auto &attr : res.attributes) {
            if (attr.label == "w1n") a = attr.value.string_value;
            if (attr.label == "w2n") b = attr.value.string_value;
            if (attr.label == "isd") is_dark = attr.value.bool_value;
            if (attr.label == "isl") is_lit = attr.value.bool_value;
          }
          if (!a.empty() && !b.empty()) {
            if (is_dark) problem_expert_->addPredicate(plansys2::Predicate("(is-dark " + a + " " + b + ")"));
            if (is_lit)  problem_expert_->addPredicate(plansys2::Predicate("(is-lit " + a + " " + b + ")"));
          }
        }
      }
    }
  }

  // ===== CONFIG TRAITS =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query =
      "match $t (configuration:$c) isa config-trait; "
      "$c has config-name $cn; "
      "$t has uses-lidar $ul; $t has uses-high-speed $uh; $t has uses-low-speed $us; "
      "fetch $cn; $ul; $uh; $us;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Config-trait query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          std::string c; bool ul = false, uh = false, us = false;
          for (const auto &attr : res.attributes) {
            if (attr.label == "cn") c = attr.value.string_value;
            if (attr.label == "ul") ul = attr.value.bool_value;
            if (attr.label == "uh") uh = attr.value.bool_value;
            if (attr.label == "us") us = attr.value.bool_value;
          }
          if (!c.empty()) {
            if (ul) problem_expert_->addPredicate(plansys2::Predicate("(uses-lidar " + c + ")"));
            if (uh) problem_expert_->addPredicate(plansys2::Predicate("(uses-high-speed " + c + ")"));
            if (us) problem_expert_->addPredicate(plansys2::Predicate("(uses-low-speed " + c + ")"));
          }
        }
      }
    }
  }

  // ===== CAN-TRAVERSE =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query =
      "match $c (from:$w1, to:$w2) isa corridor; $w1 has waypoint-name $w1n; $w2 has waypoint-name $w2n; "
      "$cfg isa configuration, has config-name $cn; "
      "fetch $w1n; $w2n; $cn;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Can-traverse query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          std::string a, b, cfg;
          for (const auto &attr : res.attributes) {
            if (attr.label == "w1n") a = attr.value.string_value;
            if (attr.label == "w2n") b = attr.value.string_value;
            if (attr.label == "cn")  cfg = attr.value.string_value;
          }
          if (!a.empty() && !b.empty() && !cfg.empty()) {
            problem_expert_->addPredicate(plansys2::Predicate("(can-traverse " + a + " " + b + " " + cfg + ")"));
          }
        }
      }
    }
  }

  // ===== BATTERY =====
  {
    double battery = 70.0; // fallback
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query =
      "match $m isa measure, has measure-name \"battery-level\", has value $v; fetch $v;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (fut.valid() && fut.wait_for(1s) == std::future_status::ready) {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          for (const auto &attr : res.attributes) {
            if (attr.label == "v") battery = attr.value.double_value;
          }
        }
      }
    }

    plansys2::Function f_batt;
    f_batt.name = "battery-level";
    f_batt.value = battery;
    f_batt.parameters = {};
    if (!problem_expert_->existFunction(f_batt))
      problem_expert_->addFunction(f_batt);
    else
      problem_expert_->updateFunction(f_batt);
  }

  // ===== DISTANCES =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query =
      "match $c (from:$w1, to:$w2) isa corridor; "
      "$w1 has waypoint-name $a; $w2 has waypoint-name $b; "
      "fetch $a; $b;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Distance query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          std::string a, b;
          for (const auto &attr : res.attributes) {
            if (attr.label == "a") a = attr.value.string_value;
            if (attr.label == "b") b = attr.value.string_value;
          }
          if (!a.empty() && !b.empty()) {
            double d = 10.0;
            plansys2::Function f;
            f.name = "distance";
            plansys2_msgs::msg::Param p1, p2;
            p1.name = a; p2.name = b;
            f.parameters = {p1, p2};
            f.value = d;
            if (!problem_expert_->existFunction(f))
              problem_expert_->addFunction(f);
            else
              problem_expert_->updateFunction(f);
          }
        }
      }
    }
  }

  // ===== ENERGY COST =====
  {
    auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    req->query_type = "fetch";
    req->query =
      "match $c (from:$w1, to:$w2) isa corridor; "
      "$e (corridor:$c) isa energy-cost, has value $cost; "
      "$w1 has waypoint-name $a; $w2 has waypoint-name $b; "
      "$cfg isa configuration, has config-name $cn; "
      "fetch $a; $b; $cn; $cost;";
    auto fut = typedb_query_cli_->async_send_request(req);

    if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Energy-cost query invalid or timed out");
    } else {
      auto resp = fut.get();
      if (resp->success) {
        for (const auto &res : resp->results) {
          std::string a, b, cn; double ec = 10.0;
          for (const auto &attr : res.attributes) {
            if (attr.label == "a") a = attr.value.string_value;
            if (attr.label == "b") b = attr.value.string_value;
            if (attr.label == "cn") cn = attr.value.string_value;
            if (attr.label == "cost") ec = attr.value.double_value;
          }
          if (!a.empty() && !b.empty() && !cn.empty()) {
            plansys2::Function f;
            f.name = "energy-cost";
            plansys2_msgs::msg::Param p1, p2, p3;
            p1.name = a; p2.name = b; p3.name = cn;
            f.parameters = {p1, p2, p3};
            f.value = ec;
            if (!problem_expert_->existFunction(f))
              problem_expert_->addFunction(f);
            else
              problem_expert_->updateFunction(f);
          }
        }
      }
    }
  }

  // ===== GOAL =====
  problem_expert_->setGoal(plansys2::Goal("(at wp_4)"));
}


 void NavigationController::execute_plan()
  {
    if (!typedb_query_cli_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "TypeDB not ready, skipping planning tick");
      return;
    }

    build_problem_from_kb();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_WARN(get_logger(), "Plan not found. Current goal: %s",
        parser::pddl::toString(problem_expert_->getGoal()).c_str());
      return;
    }

    for (const auto &item : plan->items) {
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
    if (first_iteration_) {
      execute_plan();
      first_iteration_ = false;
      return;
    }

    // Example: if some typedb condition tells you to re-evaluate
    // 1) Refresh problem
    // build_problem_from_kb();
    // 2) Trigger replan
    // system("ros2 service call /executor/replan_for_execution std_srvs/srv/Trigger {}");

    // Continue monitoring action feedback and errors as you already do
    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
      auto result = executor_client_->getResult();
      if (result.has_value()) {
        RCLCPP_INFO(this->get_logger(), "Plan execution finished");
        finish_controlling();
        return;
      } else {
        RCLCPP_INFO(this->get_logger(), "Replanning");
        execute_plan();
        return;
      }
    }

    auto feedback = executor_client_->getFeedBack();
    for (const auto &ae : feedback.action_execution_status) {
      if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
        RCLCPP_ERROR(this->get_logger(), "[%s] failed: %s",
                    ae.action.c_str(), ae.message_status.c_str());
        // Update problem from KB then replan
        build_problem_from_kb();
        system("ros2 service call /executor/replan_for_execution std_srvs/srv/Trigger {}");
        return;
      }
    }
  }





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

