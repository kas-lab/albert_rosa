#include <fstream>
#include <limits>
#include <future>
#include <algorithm>
#include <sstream>

#include "navigate.hpp"

using namespace std::chrono_literals;
using navigation_task_plan::NavigationController;

namespace navigation_task_plan
{

NavigationController::NavigationController(const std::string & node_name)
: rclcpp::Node(node_name)
{
  // PlanSys2 clients
  domain_expert_  = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>("rosa_plansys_controller_executor");

  // Periodic planning tick
  step_timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  step_timer_ = this->create_wall_timer(1s, std::bind(&NavigationController::step, this), step_timer_cb_group_);

  // ROSA KB query client
  ros_typedb_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  typedb_query_cli_ = this->create_client<ros_typedb_msgs::srv::Query>(
      "/rosa_kb/query", rclcpp::QoS(rclcpp::ServicesQoS()), ros_typedb_cb_group_);

  // Optional: Battery subscription (debug only)
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", rclcpp::QoS(10),
      std::bind(&NavigationController::batteryCallback, this, std::placeholders::_1));

  rosa_event_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/rosa_kb/events", 10);
  
  RCLCPP_INFO(get_logger(), "NavigationController initialized with ROSA integration.");

// Optional: Subscribe to ROSA feedback to know when adaptation completes
  rosa_reconfig_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/rosa_kb/events", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data == "reconfiguration_completed") {
        adaptation_triggered_ = false;  // Reset flag
        RCLCPP_INFO(get_logger(), "✅ ROSA reconfiguration completed");
      }
    });

  // this->declare_parameter("enable_reactive", false);
  this->declare_parameter("enable_proactive", true);
  
  // bool enable_reactive = this->get_parameter("enable_reactive").as_bool();
  bool enable_proactive = this->get_parameter("enable_proactive").as_bool();
  
  // if (enable_reactive) {
  //   proactive_timer_ = this->create_wall_timer(
  //       3s, std::bind(&NavigationController::evaluatePlanFeasibility, this));
  //   RCLCPP_INFO(get_logger(), "Reactive monitoring ENABLED");
  // }

  if (enable_proactive) {
    future_timer_ = this->create_wall_timer(
        5s, std::bind(&NavigationController::evaluateFutureFeasibility, this));
    RCLCPP_INFO(get_logger(), "Proactive reasoning ENABLED");
  }

  RCLCPP_INFO(get_logger(), "NavigationController initialized.");
}

NavigationController::~NavigationController() = default;

// -----------------------------------------------------------------------------
// Build PDDL problem from KB
// -----------------------------------------------------------------------------
void NavigationController::build_problem_from_kb()
{
  problem_expert_->clearKnowledge();
  cost_map_.clear();
  
  fetch_waypoints();
  fetch_corridors();
  fetch_configurations();
  fetch_lighting_conditions();
  fetch_energy_costs();
  fetch_battery_and_feasibility();
  fetch_goal();

  problem_expert_->addPredicate(plansys2::Predicate("(at wp_0)"));

  std::ofstream out("/tmp/runtime_problem.pddl");
  out << problem_expert_->getProblem();
  out.close();
  RCLCPP_INFO(get_logger(), "Problem saved to /tmp/runtime_problem.pddl");
  
  // Debug output
  RCLCPP_INFO(get_logger(), "cost_map_ now has %zu entries", cost_map_.size());
}

// -----------------------------------------------------------------------------
// KB fetchers
// -----------------------------------------------------------------------------
void NavigationController::fetch_waypoints()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $wp isa waypoint, has waypoint-name $name; fetch $name;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(5s) != std::future_status::ready) return;

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &res : resp->results)
    for (const auto &attr : res.attributes)
      if (attr.label == "waypoint-name")
        problem_expert_->addInstance(plansys2::Instance(attr.value.string_value, "waypoint"));
}

void NavigationController::fetch_corridors()
{
  corridor_pairs_.clear();

  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match $c (from:$w1, to:$w2) isa corridor; "
    "$w1 has waypoint-name $a; $w2 has waypoint-name $b; "
    "fetch $a; $b;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) return;

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &row : resp->results) {
    std::string a, b;
    for (const auto &attr : row.attributes) {
      if (attr.name == "a") a = attr.value.string_value;
      if (attr.name == "b") b = attr.value.string_value;
    }
    if (!a.empty() && !b.empty()) {
      corridor_pairs_.emplace_back(a, b);
      problem_expert_->addPredicate(plansys2::Predicate("(is-corridor " + a + " " + b + ")"));
    }
  }
}

void NavigationController::fetch_configurations()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $c isa component-configuration, has component-configuration-name $n; fetch $n;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) return;

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &res : resp->results)
    for (const auto &attr : res.attributes)
      if (attr.name == "n") {
        const auto cfg = attr.value.string_value;
        problem_expert_->addInstance(plansys2::Instance(cfg, "configuration"));
        problem_expert_->addPredicate(plansys2::Predicate("(can-use " + cfg + ")"));
        problem_expert_->addPredicate(plansys2::Predicate("(config-valid " + cfg + ")"));
      }

  for (const auto &[w1, w2] : corridor_pairs_)
    for (const auto &cfg : {"high_speed_config", "low_speed_config"})
      problem_expert_->addPredicate(plansys2::Predicate(
        "(can-traverse " + w1 + " " + w2 + " " + std::string(cfg) + ")"));
}

void NavigationController::fetch_lighting_conditions()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match $l (from:$w1, to:$w2) isa lighting-condition, has is-dark $isd, has is-lit $isl; "
    "$w1 has waypoint-name $a; $w2 has waypoint-name $b; fetch $a; $b; $isd; $isl;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) return;

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &row : resp->results) {
    std::string a, b;
    bool is_dark = false, is_lit = false;
    for (const auto &x : row.attributes) {
      if (x.name == "a") a = x.value.string_value;
      if (x.name == "b") b = x.value.string_value;
      if (x.name == "isd") is_dark = x.value.bool_value;
      if (x.name == "isl") is_lit = x.value.bool_value;
    }
    if (a.empty() || b.empty()) continue;
    if (is_dark) problem_expert_->addPredicate(plansys2::Predicate("(is-dark " + a + " " + b + ")"));
    if (is_lit)  problem_expert_->addPredicate(plansys2::Predicate("(is-lit " + a + " " + b + ")"));
  }
}

void NavigationController::fetch_energy_costs()
{
  RCLCPP_INFO(get_logger(), "Fetching energy costs from KB...");
  
  // Simplified approach: get corridor costs without config link
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match $corr (from:$w1, to:$w2) isa corridor; "
    "$e (corridor:$corr) isa energy-cost, has value $cost; "
    "$w1 has waypoint-name $a; $w2 has waypoint-name $b; "
    "fetch $a; $b; $cost;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Timeout fetching energy costs");
    return;
  }

  auto resp = fut.get();
  if (!resp->success) {
    RCLCPP_ERROR(get_logger(), "Failed to fetch energy costs");
    return;
  }

  // Store corridor costs
  std::map<std::pair<std::string, std::string>, double> corridor_costs;
  
  for (const auto &row : resp->results) {
    std::string a, b;
    double ec = 0.0;
    for (const auto &x : row.attributes) {
      if (x.name == "a") a = x.value.string_value;
      if (x.name == "b") b = x.value.string_value;
      if (x.name == "cost") ec = x.value.double_value;
    }
    if (a.empty() || b.empty()) continue;
    
    corridor_costs[{a, b}] = ec;
    RCLCPP_INFO(get_logger(), "  Corridor %s→%s: cost=%.2f", a.c_str(), b.c_str(), ec);
  }

  if (corridor_costs.empty()) {
    RCLCPP_ERROR(get_logger(), "No corridor costs found in KB!");
    return;
  }

  // Get all configurations
  std::vector<std::string> configs = {"high_speed_config", "low_speed_config"};
  
  // Populate cost_map_ for each (corridor, config) pair
  for (const auto &[wp_pair, base_cost] : corridor_costs) {
    for (const auto &cfg : configs) {
      auto key = std::make_tuple(wp_pair.first, wp_pair.second, cfg);
      cost_map_[key] = base_cost;

      // Also add to PlanSys2
      plansys2_msgs::msg::Node node;
      node.node_type = plansys2_msgs::msg::Node::FUNCTION;
      node.name = "energy-cost";
      node.value = base_cost;

      plansys2_msgs::msg::Param p1, p2, p3;
      p1.name = wp_pair.first; 
      p2.name = wp_pair.second; 
      p3.name = cfg;
      node.parameters = {p1, p2, p3};

      plansys2::Function f(node);
      if (!problem_expert_->existFunction(f))
        problem_expert_->addFunction(f);
      else
        problem_expert_->updateFunction(f);
    }
  }
  
  RCLCPP_INFO(get_logger(), 
    "✓ Loaded %zu corridor costs → %zu cost_map_ entries",
    corridor_costs.size(), cost_map_.size());
}

std::vector<std::string> NavigationController::getFeasibleConfigsFromKB()
{
  std::vector<std::string> out;

  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match $cfg isa component-configuration, "
    "has component-configuration-name $n, "
    "has component-configuration-status 'feasible'; fetch $n;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(800ms) != std::future_status::ready) return out;
  auto resp = fut.get();
  if (!resp->success) return out;

  for (const auto &row : resp->results)
    for (const auto &a : row.attributes)
      if (a.name == "n") out.emplace_back(a.value.string_value);

  return out;
}

void NavigationController::fetch_battery_and_feasibility()
{
  battery_level_ = getBatteryFromKB();
  plansys2::Function batt;
  batt.name = "battery-level";
  batt.value = battery_level_;
  if (!problem_expert_->existFunction(batt)) problem_expert_->addFunction(batt);
  else problem_expert_->updateFunction(batt);

  const auto feasible_cfgs = getFeasibleConfigsFromKB();
  for (const auto &[w1, w2] : corridor_pairs_)
    for (const auto &cfg : feasible_cfgs)
      problem_expert_->addPredicate(plansys2::Predicate(
        "(has-enough-battery " + w1 + " " + w2 + " " + cfg + ")"));
}

void NavigationController::fetch_goal()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $g isa goal, has goal-name $gn; fetch $gn;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) return;

  auto resp = fut.get();
  if (!resp->success || resp->results.empty()) return;

  std::string goal_name;
  for (const auto &res : resp->results)
    for (const auto &attr : res.attributes)
      if (attr.name == "gn") goal_name = attr.value.string_value;

  if (!goal_name.empty())
    problem_expert_->setGoal(plansys2::Goal("(and (at " + goal_name + "))"));
}

// -----------------------------------------------------------------------------
// Adaptation & Feasibility
// -----------------------------------------------------------------------------
void NavigationController::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  double pct = msg->percentage <= 1.0 ? msg->percentage * 100.0 : msg->percentage;
  RCLCPP_DEBUG(get_logger(), "Battery topic hint: %.2f%%", pct);
}

double NavigationController::getBatteryFromKB()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match $r_meas (measured-attribute: $b) isa measurement, "
    "has latest true, has measurement-value $v; "
    "$b isa QualityAttribute, has measure-name 'battery-level'; fetch $v;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(800ms) != std::future_status::ready) return battery_level_;

  auto resp = fut.get();
  if (!resp->success || resp->results.empty()) return battery_level_;

  for (const auto &row : resp->results)
    for (const auto &a : row.attributes)
      if (a.name == "v") return a.value.double_value;

  return battery_level_;
}

// void NavigationController::triggerProactiveAdaptation(const std::string &target_cfg)
// {
//   RCLCPP_WARN(get_logger(), "🔄 Requesting configuration change to '%s'...", target_cfg.c_str());
  
//   auto client = this->create_client<rosa_msgs::srv::SelectedConfigurations>(
//     "/rosa_kb/select_configuration");

//   if (!client->wait_for_service(std::chrono::seconds(3))) {
//     RCLCPP_ERROR(get_logger(), "ROSA configuration planner not available.");
//     return;
//   }

//   auto req = std::make_shared<rosa_msgs::srv::SelectedConfigurations::Request>();
//   rosa_msgs::msg::ComponentConfiguration config;
//   config.name = target_cfg;
//   config.component.name = "controller_server";
//   config.status = "selected";
//   config.priority = 1.0;
//   req->selected_component_configs.push_back(config);

//   // MAKE IT SYNCHRONOUS - WAIT FOR RESPONSE
//   auto future = client->async_send_request(req);
  
//   if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
//     auto resp = future.get();
//     if (resp->success) {
//       RCLCPP_INFO(get_logger(), "✅ ROSA switched to '%s'", target_cfg.c_str());
//       current_config_ = target_cfg;
      
//       // Give Nav2 time to reconfigure
//       std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     } else {
//       RCLCPP_ERROR(get_logger(), "❌ ROSA failed to switch configuration");
//     }
//   } else {
//     RCLCPP_ERROR(get_logger(), "⏱️  Timeout waiting for configuration change");
//   }
// }
void NavigationController::triggerProactiveAdaptation(const std::string &reason)
{
  RCLCPP_WARN(get_logger(), "🔄 ROSA adaptation triggered: %s", reason.c_str());
  
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "insert_monitoring_data";
  rosa_event_pub_->publish(std::move(msg));
  
  // Wait for ROSA to reconfigure (give executor time)
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  RCLCPP_INFO(get_logger(), "✅ ROSA adaptation request complete");
}
// void NavigationController::triggerProactiveAdaptation(const std::string &target_cfg)
// {
//   RCLCPP_WARN(get_logger(), "🔄 Directly changing Nav2 parameter for '%s'...", target_cfg.c_str());
  
//   // Create parameter client for controller_server
//   auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
//     this, "/controller_server");
  
//   if (!param_client->wait_for_service(std::chrono::seconds(2))) {
//     RCLCPP_ERROR(get_logger(), "controller_server not available!");
//     return;
//   }

//   // Set speed based on config
//   double target_speed = (target_cfg == "low_speed_config") ? 0.4 : 0.8;
  
//   auto result = param_client->set_parameters({
//     rclcpp::Parameter("FollowPath.max_vel_x", target_speed)
//   });
  
//   // Wait for it to complete
//   if (result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
//     RCLCPP_INFO(get_logger(), "✅ Changed max_vel_x to %.2f m/s", target_speed);
//     current_config_ = target_cfg;
//   } else {
//     RCLCPP_ERROR(get_logger(), "❌ Timeout setting parameter");
//   }
// }

void NavigationController::triggerReplan()
{
  RCLCPP_WARN(get_logger(), "Triggering proactive replan due to predicted infeasibility...");

  build_problem_from_kb();

  auto domain  = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan    = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    RCLCPP_ERROR(get_logger(), "No new plan found during replan attempt.");
    return;
  }

  executor_client_->cancel_plan_execution();
  executor_client_->start_plan_execution(plan.value());
  RCLCPP_INFO(get_logger(), "New plan generated and execution restarted.");
}

// -----------------------------------------------------------------------------
// Reactive check
// -----------------------------------------------------------------------------
void NavigationController::evaluatePlanFeasibility()
{
  battery_level_ = getBatteryFromKB();
  const auto feasible_cfgs = getFeasibleConfigsFromKB();

  if (feasible_cfgs.empty()) {
    RCLCPP_WARN(get_logger(),
      "No feasible configs at battery %.2f%% — triggering replan.", battery_level_);
    triggerReplan();
    return;
  }

  if (std::find(feasible_cfgs.begin(), feasible_cfgs.end(), current_config_) == feasible_cfgs.end()) {
    RCLCPP_WARN(get_logger(),
      "Current config '%s' no longer feasible (battery %.2f). Triggering adaptation.",
      current_config_.c_str(), battery_level_);
    triggerProactiveAdaptation(feasible_cfgs.front());
  } else {
    RCLCPP_INFO(get_logger(),
      "Feasible: Battery %.1f%% | Config '%s' valid.",
      battery_level_, current_config_.c_str());
  }
}

// -----------------------------------------------------------------------------
// Helper: parse action string
// -----------------------------------------------------------------------------
std::tuple<std::string, std::string, std::string>
NavigationController::parse_action(const std::string &action)
{
  // Expected format: "(navigate wp_0 wp_1 high_speed_config)"
  std::stringstream ss(action);
  std::string action_name, from, to, cfg;
  
  ss >> action_name >> from >> to >> cfg;

  // Remove parentheses
  auto clean = [](std::string &s) {
    s.erase(std::remove(s.begin(), s.end(), '('), s.end());
    s.erase(std::remove(s.begin(), s.end(), ')'), s.end());
  };

  clean(action_name);
  clean(from);
  clean(to);
  clean(cfg);

  if (from.empty() || to.empty() || cfg.empty()) {
    RCLCPP_ERROR(get_logger(), 
      "Failed to parse action: '%s' -> action=%s, from='%s', to='%s', cfg='%s'",
      action.c_str(), action_name.c_str(), from.c_str(), to.c_str(), cfg.c_str());
  }

  return {from, to, cfg};
}

// -----------------------------------------------------------------------------
// Proactive check
// -----------------------------------------------------------------------------
void NavigationController::evaluateFutureFeasibility()
{
  battery_level_ = getBatteryFromKB();
  auto feedback = executor_client_->getFeedBack();
  
  if (feedback.action_execution_status.empty()) {
    RCLCPP_DEBUG(get_logger(), "[Proactive] No active plan to evaluate");
    return;
  }

  if (cost_map_.empty()) {
    RCLCPP_WARN(get_logger(), "[Proactive] cost_map_ is empty! Skipping check.");
    return;
  }

  if (current_plan_actions_.empty()) {
    RCLCPP_WARN(get_logger(), "[Proactive] No stored plan actions! Skipping check.");
    return;
  }

  double total_predicted_cost = 0.0;
  int actions_counted = 0;

  RCLCPP_INFO(get_logger(), "\n=== Proactive Feasibility Check ===");
  RCLCPP_INFO(get_logger(), "Current battery: %.2f%%", battery_level_);
  RCLCPP_INFO(get_logger(), "Plan has %zu actions, %zu in execution feedback", 
              current_plan_actions_.size(), feedback.action_execution_status.size());

  // Match execution status with our stored plan
  for (size_t i = 0; i < feedback.action_execution_status.size() && 
                     i < current_plan_actions_.size(); i++) {
    const auto &ae = feedback.action_execution_status[i];
    const auto &pa = current_plan_actions_[i];
    
    // Only count future actions (not yet completed)
    if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED ||
        ae.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
      
      total_predicted_cost += pa.cost;
      actions_counted++;
      
      RCLCPP_INFO(get_logger(), 
        "  [%zu] %s: %s→%s (%s) cost=%.2f | Status=%d", 
        i, pa.action_name.c_str(), pa.from.c_str(), pa.to.c_str(), 
        pa.config.c_str(), pa.cost, ae.status);
    } else {
      RCLCPP_DEBUG(get_logger(), 
        "  [%zu] %s: COMPLETED (status=%d)", 
        i, pa.action_name.c_str(), ae.status);
    }
  }

  RCLCPP_INFO(get_logger(), "-----------------------------------");
  RCLCPP_INFO(get_logger(), "Remaining actions: %d", actions_counted);
  RCLCPP_INFO(get_logger(), "Total predicted cost: %.2f%%", total_predicted_cost);
  RCLCPP_INFO(get_logger(), "Current battery: %.2f%%", battery_level_);
  RCLCPP_INFO(get_logger(), "===================================\n");

  if (total_predicted_cost <= 0.0) {
    RCLCPP_DEBUG(get_logger(), "[Proactive] No remaining cost to evaluate");
    return;
  }

  double remaining = battery_level_ - total_predicted_cost;

  // ✅ KEEP YOUR MONITORING LOGIC - just change the trigger
  if (remaining < safety_margin_) {
    // Guard to prevent repeated triggers
    if (!adaptation_triggered_) {  // ← Add this flag
      RCLCPP_WARN(get_logger(),
        "[🚨 PROACTIVE REASONING TRIGGERED]\n"
        "  Current battery: %.2f%%\n"
        "  Predicted cost: %.2f%%\n"
        "  Battery after plan: %.2f%%\n"
        "  Safety margin: %.2f%%\n"
        "  → Triggering ROSA adaptation",
        battery_level_, total_predicted_cost, remaining, safety_margin_);

      // ❌ OLD: triggerProactiveAdaptation("low_speed_config");
      // ✅ NEW: Publish event for ROSA
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = "insert_monitoring_data";
      rosa_event_pub_->publish(std::move(msg));
      
      adaptation_triggered_ = true;  // Prevent re-triggering
      RCLCPP_INFO(get_logger(), "✅ Adaptation event published to ROSA");
      
    } else {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000,
        "⏳ Adaptation already triggered, waiting for ROSA...");
    }
  } else {
    // ✅ Reset flag when we're back to feasible
    adaptation_triggered_ = false;
    
    RCLCPP_INFO(get_logger(),
      "[✓ Proactive Check] Plan FEASIBLE\n"
      "  Battery after plan: %.2f%% (margin: %.2f%%)",
      remaining, remaining - safety_margin_);
  }
}

// -----------------------------------------------------------------------------
// PlanSys2 execution loop
// -----------------------------------------------------------------------------
void NavigationController::execute_plan()
{
  if (!typedb_query_cli_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "ROSA KB not ready; skip planning tick");
    return;
  }

  build_problem_from_kb();

  auto domain  = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan    = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    RCLCPP_WARN(get_logger(), "No plan available. Goal: %s",
      parser::pddl::toString(problem_expert_->getGoal()).c_str());
    return;
  }

  // Clear and rebuild plan action list
  current_plan_actions_.clear();
  
  RCLCPP_INFO(get_logger(), "\n=== GENERATED PLAN ===");
  for (size_t i = 0; i < plan->items.size(); i++) {
    const auto &item = plan->items[i];
    RCLCPP_INFO(get_logger(), "[%zu] Action: '%s'", i, item.action.c_str());
    
    auto [from, to, cfg] = parse_action(item.action);
    
    if (!from.empty() && !to.empty() && !cfg.empty()) {
      std::stringstream ss(item.action);
      std::string action_name;
      ss >> action_name;
      action_name.erase(std::remove(action_name.begin(), action_name.end(), '('), action_name.end());
      action_name.erase(std::remove(action_name.begin(), action_name.end(), ')'), action_name.end());
      
      auto key = std::make_tuple(from, to, cfg);
      double cost = 0.0;
      if (cost_map_.count(key)) {
        cost = cost_map_[key];
      }
      
      ParsedAction pa;
      pa.action_name = action_name;
      pa.from = from;
      pa.to = to;
      pa.config = cfg;
      pa.cost = cost;
      
      current_plan_actions_.push_back(pa);
      
      RCLCPP_INFO(get_logger(), 
        "     ✓ %s: %s → %s (%s) cost=%.2f", 
        action_name.c_str(), from.c_str(), to.c_str(), cfg.c_str(), cost);
    } else {
      RCLCPP_WARN(get_logger(), "     ✗ Failed to parse action");
    }
  }
  
  // ✅ FIX: Update current_config_ to match the plan!
  if (!current_plan_actions_.empty()) {
    current_config_ = current_plan_actions_[0].config;
    RCLCPP_INFO(get_logger(), 
      "📌 Plan uses configuration: '%s'", current_config_.c_str());
  }
  
  RCLCPP_INFO(get_logger(), "======================\n");

  executor_client_->start_plan_execution(plan.value());
}

void NavigationController::finish_controlling()
{
  step_timer_->cancel();
  executor_client_->cancel_plan_execution();
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
      RCLCPP_INFO(get_logger(), "Plan execution finished successfully.");
      finish_controlling();
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Replanning after failed execution step.");
      execute_plan();
      return;
    }
  }

  auto feedback = executor_client_->getFeedBack();
  for (const auto &ae : feedback.action_execution_status) {
    if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      RCLCPP_ERROR(get_logger(), "[%s] failed: %s",
        ae.action.c_str(), ae.message_status.c_str());
      build_problem_from_kb();

      // Trigger internal PlanSys2 replan service for runtime recovery
      system("ros2 service call /executor/replan_for_execution std_srvs/srv/Trigger {}");
      return;
    }
  }
}

}  // namespace navigation_task_plan

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_task_plan::NavigationController>("navigate_controller");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}