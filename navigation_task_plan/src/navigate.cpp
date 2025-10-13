#include <ctime>
#include <fstream> 
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

  step_timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  step_timer_ = this->create_wall_timer(1s, std::bind(&NavigationController::step, this), step_timer_cb_group_);

  ros_typedb_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  typedb_query_cli_ = this->create_client<ros_typedb_msgs::srv::Query>(
      "ros_typedb/query", rclcpp::QoS(rclcpp::ServicesQoS()), ros_typedb_cb_group_);

  // --- Proactive adaptation loop initialization ---
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10, std::bind(&NavigationController::batteryCallback, this, _1));

  proactive_timer_ = this->create_wall_timer(
      3s, std::bind(&NavigationController::evaluatePlanFeasibility, this));

  RCLCPP_INFO(this->get_logger(), "Proactive adaptation loop initialized.");

}

NavigationController::~NavigationController() = default;

void NavigationController::build_problem_from_kb()
{
  problem_expert_->clearKnowledge();

  // Each part builds and updates its portion of the PDDL problem
  fetch_waypoints();
  fetch_corridors();
  fetch_configurations();
  fetch_config_traits();
  fetch_lighting_conditions();
  fetch_energy_costs();
  fetch_battery_and_feasibility();
  fetch_goal();

  // Add initial position and save problem file
  problem_expert_->addPredicate(plansys2::Predicate("(at wp_0)"));
  auto problem_text = problem_expert_->getProblem();
  std::ofstream out("/tmp/runtime_problem.pddl");
  out << problem_text;
  out.close();
  RCLCPP_INFO(get_logger(), "Problem saved to /tmp/runtime_problem.pddl");
}

// ============================================================================
// ============ INDIVIDUAL QUERY FUNCTIONS ====================================
// ============================================================================

void NavigationController::fetch_waypoints()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $wp isa waypoint, has waypoint-name $name; fetch $name;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Waypoint query invalid or timed out");
    return;
  }

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &res : resp->results) {
    for (const auto &attr : res.attributes) {
      if (attr.label == "waypoint-name")
        problem_expert_->addInstance(plansys2::Instance(attr.value.string_value, "waypoint"));
    }
  }
}

void NavigationController::fetch_corridors()
{
  corridor_pairs_.clear();

  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match "
    "  $c (from:$w1, to:$w2) isa corridor; "
    "  $w1 has waypoint-name $w1-name; "
    "  $w2 has waypoint-name $w2-name; "
    "fetch $w1-name; $w2-name;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Corridor query timed out");
    return;
  }

  auto result = fut.get();
  for (const auto &entry : result->results) {
    std::string w1_name, w2_name;
    for (const auto &attr : entry.attributes) {
      if (attr.name == "w1-name") w1_name = attr.value.string_value;
      else if (attr.name == "w2-name") w2_name = attr.value.string_value;
    }

    if (!w1_name.empty() && !w2_name.empty()) {
      corridor_pairs_.emplace_back(w1_name, w2_name);
      auto predicate = "(is-corridor " + w1_name + " " + w2_name + ")";
      problem_expert_->addPredicate(plansys2::Predicate(predicate));
    }
  }
}

void NavigationController::fetch_configurations()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $c isa configuration, has config-name $n; fetch $n;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Configuration query invalid or timed out");
    return;
  }

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &res : resp->results) {
    for (const auto &attr : res.attributes) {
      if (attr.name == "n") {
        const auto cfg = attr.value.string_value;
        problem_expert_->addInstance(plansys2::Instance(cfg, "configuration"));
        problem_expert_->addPredicate(plansys2::Predicate("(can-use " + cfg + ")"));
        problem_expert_->addPredicate(plansys2::Predicate("(config-valid " + cfg + ")"));
      }
    }
  }

  // Add can-traverse predicates
  for (const auto &[w1, w2] : corridor_pairs_) {
    for (const auto &cfg : {"config1", "config2", "config3", "config4"}) {
      problem_expert_->addPredicate(plansys2::Predicate(
        "(can-traverse " + w1 + " " + w2 + " " + std::string(cfg) + ")"));
    }
  }
}

void NavigationController::fetch_config_traits()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match "
    "  $ct (configuration:$c) isa config-trait, "
    "      has uses-lidar $lidar, "
    "      has uses-high-speed $high, "
    "      has uses-low-speed $low; "
    "  $c has config-name $name; "
    "fetch $name; $lidar; $high; $low;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Config-trait query invalid or timed out");
    return;
  }

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &res : resp->results) {
    std::string cfg; bool lidar = false, high = false, low = false;
    for (const auto &attr : res.attributes) {
      if (attr.name == "name") cfg = attr.value.string_value;
      if (attr.name == "lidar") lidar = attr.value.bool_value;
      if (attr.name == "high")  high  = attr.value.bool_value;
      if (attr.name == "low")   low   = attr.value.bool_value;
    }

    if (!cfg.empty()) {
      if (lidar) problem_expert_->addPredicate(plansys2::Predicate("(uses-lidar " + cfg + ")"));
      if (high)  problem_expert_->addPredicate(plansys2::Predicate("(uses-high-speed " + cfg + ")"));
      if (low)   problem_expert_->addPredicate(plansys2::Predicate("(uses-low-speed " + cfg + ")"));
    }
  }
}

void NavigationController::fetch_lighting_conditions()
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
    return;
  }

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &res : resp->results) {
    std::string a, b; bool is_dark = false, is_lit = false;
    for (const auto &attr : res.attributes) {
      if (attr.name == "w1n") a = attr.value.string_value;
      if (attr.name == "w2n") b = attr.value.string_value;
      if (attr.name == "isd") is_dark = attr.value.bool_value;
      if (attr.name == "isl") is_lit  = attr.value.bool_value;
    }
    if (!a.empty() && !b.empty()) {
      if (is_dark) problem_expert_->addPredicate(plansys2::Predicate("(is-dark " + a + " " + b + ")"));
      if (is_lit)  problem_expert_->addPredicate(plansys2::Predicate("(is-lit "  + a + " " + b + ")"));
    }
  }
}

void NavigationController::fetch_energy_costs()
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
    return;
  }

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &res : resp->results) {
    std::string a, b, cn; double ec = 10.0;
    for (const auto &attr : res.attributes) {
      if (attr.name == "a") a = attr.value.string_value;
      if (attr.name == "b") b = attr.value.string_value;
      if (attr.name == "cn") cn = attr.value.string_value;
      if (attr.name == "cost") ec = attr.value.double_value;
    }
    if (!a.empty() && !b.empty() && !cn.empty()) {
      plansys2_msgs::msg::Node node;
      node.node_type = plansys2_msgs::msg::Node::FUNCTION;
      node.name = "energy-cost";
      plansys2_msgs::msg::Param p1, p2, p3;
      p1.name = a; p2.name = b; p3.name = cn;
      node.parameters = {p1, p2, p3};
      node.value = ec;

      plansys2::Function func(node);
      if (!problem_expert_->existFunction(func))
        problem_expert_->addFunction(func);
      else
        problem_expert_->updateFunction(func);
    }
  }
}

void NavigationController::fetch_battery_and_feasibility()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $m isa measure, has measure-name 'battery-level', has value $v; fetch $v;";

  double battery_now = 100.0;
  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.valid() && fut.wait_for(1s) == std::future_status::ready) {
    auto resp = fut.get();
    if (resp->success && !resp->results.empty()) {
      for (const auto &res : resp->results) {
        for (const auto &attr : res.attributes) {
          if (attr.name == "v")
            battery_now = attr.value.double_value;
        }
      }
      RCLCPP_INFO(get_logger(), "Fetched battery-level = %.2f", battery_now);
    }
  }

  for (const auto &[w1, w2] : corridor_pairs_) {
    for (const auto &cfg : {"config1", "config2", "config3", "config4"}) {
      std::string func_key = "(energy-cost " + w1 + " " + w2 + " " + std::string(cfg) + ")";
      auto maybe_func = problem_expert_->getFunction(func_key);
      double energy_cost = maybe_func.has_value() ? maybe_func.value().value : std::numeric_limits<double>::infinity();
      if (battery_now >= energy_cost) {
        problem_expert_->addPredicate(plansys2::Predicate("(has-enough-battery " + w1 + " " + w2 + " " + std::string(cfg) + ")"));
      }
    }
  }

  plansys2::Function batt;
  batt.name = "battery-level";
  batt.parameters = {};
  batt.value = battery_now;
  if (!problem_expert_->existFunction(batt))
    problem_expert_->addFunction(batt);
  else
    problem_expert_->updateFunction(batt);
}

void NavigationController::fetch_goal()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $g isa goal, has goal-name $gn; fetch $gn;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (!fut.valid() || fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Goal query invalid or timed out");
    return;
  }

  auto resp = fut.get();
  if (!resp->success || resp->results.empty()) return;

  std::string goal_name;
  for (const auto &res : resp->results)
    for (const auto &attr : res.attributes)
      if (attr.name == "gn") goal_name = attr.value.string_value;

  if (!goal_name.empty()) {
    problem_expert_->setGoal(plansys2::Goal("(and (at " + goal_name + "))"));
    RCLCPP_INFO(get_logger(), "Fetched goal from KB: (at %s)", goal_name.c_str());
  }
}
void NavigationController::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  battery_level_ = msg->percentage;
  RCLCPP_DEBUG(this->get_logger(), "Battery updated: %.2f%%", battery_level_);
}

double NavigationController::computePredictedCost()
{
  // Later, replace this with a query to TypeDB to get energy-cost values
  return 35.0;  // static test value (%)
}

void NavigationController::triggerProactiveAdaptation()
{
  RCLCPP_WARN(this->get_logger(),
    "Proactive adaptation triggered: battery %.1f < predicted %.1f + margin %.1f",
    battery_level_, predicted_cost_, safety_margin_);

  // --- Create isolated node for Nav2 parameter change ---
  auto param_node = std::make_shared<rclcpp::Node>("nav2_param_client");
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
      param_node,
      "/controller_server"
  );

  if (param_client->wait_for_service(std::chrono::seconds(3))) {
    try {
      param_client->set_parameters({
        rclcpp::Parameter("FollowPath.max_vel_x", 0.2)
      });
      RCLCPP_INFO(this->get_logger(),
        "Successfully reduced Nav2 speed parameter to 0.2 m/s");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to set Nav2 parameter: %s", e.what());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
      "Nav2 controller_server not available for parameter update.");
  }

  // --- Trigger PlanSys2 replan ---
  // auto replan_client = this->create_client<std_srvs::srv::Trigger>(
  //     "/executor/replan_for_execution");
  // auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // if (replan_client->wait_for_service(std::chrono::seconds(2))) {
  //   replan_client->async_send_request(request);
  //   RCLCPP_INFO(this->get_logger(), "Requested PlanSys2 replan.");
  // } else {
  //   RCLCPP_ERROR(this->get_logger(),
  //     "Could not contact PlanSys2 replan service.");
  // }
}


void NavigationController::evaluatePlanFeasibility()
{
  predicted_cost_ = computePredictedCost();

  if ((battery_level_ - predicted_cost_) < safety_margin_) {
    triggerProactiveAdaptation();
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Feasible: Battery %.1f%% | Predicted Cost %.1f%% | Margin %.1f%%",
      battery_level_, predicted_cost_, safety_margin_);
  }
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

  for (const auto &item : plan->items)
    RCLCPP_INFO(this->get_logger(), "  Action: '%s'", item.action.c_str());

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
      build_problem_from_kb();
      system("ros2 service call /executor/replan_for_execution std_srvs/srv/Trigger {}");
      return;
    }
  }
}

} // namespace navigation_task_plan

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_task_plan::NavigationController>("navigate_controller");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
