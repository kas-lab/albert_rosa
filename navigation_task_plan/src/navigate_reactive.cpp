#include <fstream>
#include <limits>
#include <future>
#include <algorithm>
#include <sstream>
#include "std_msgs/msg/int32.hpp"
#include "navigate.hpp"
#include <random>
#include <sstream>
#include <iomanip>  // for std::setprecision
using namespace std::chrono_literals;
using navigation_task_plan::NavigationController;

struct ScenarioDifficulty {
  double nodes_skip;
  double edges_remove;
  int min_chargers;
  int max_chargers;
  std::string name;
};

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
    "/rosa_kb/query", rmw_qos_profile_services_default, ros_typedb_cb_group_);

  // Optional: Battery subscription (debug only)
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", rclcpp::QoS(10),
      std::bind(&NavigationController::batteryCallback, this, std::placeholders::_1));

  rosa_event_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/rosa_kb/events", 10);

  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);

  
  monitoring_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
  "/navigation/monitoring", 10);  // benchmark_monitor listens here

  amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      amcl_pose_ = *msg;
      latest_amcl_pose_received_ = true;
    });
  last_config_change_time_ = std::chrono::steady_clock::now(); 
  // this->declare_parameter("enable_reactive", false);
  
  // // bool enable_reactive = this->get_parameter("enable_reactive").as_bool();


  this->declare_parameter("enable_proactive", true);
  bool enable_proactive = this->get_parameter("enable_proactive").as_bool();
  enable_proactive_ = enable_proactive;

  if (enable_proactive_) {
    RCLCPP_INFO(get_logger(), "Proactive reasoning ENABLED (event-driven)");
  } else {
    RCLCPP_INFO(get_logger(), "Proactive reasoning DISABLED");
  }
  this->declare_parameter("benchmark_mode", false);
  this->declare_parameter("max_benchmark_runs", 100);
  this->declare_parameter("enable_debug_in_benchmark", false);
  benchmark_mode_ = this->get_parameter("benchmark_mode").as_bool();
  max_runs_ = this->get_parameter("max_benchmark_runs").as_int();
  bool enable_debug = this->get_parameter("enable_debug_in_benchmark").as_bool();  // ✅ NEW
  this->declare_parameter("scenario_difficulty", "easy");
  scenario_difficulty_ = this->get_parameter("scenario_difficulty").as_string();
  
  RCLCPP_INFO(get_logger(), 
    "🗺️ Map generation difficulty: %s", scenario_difficulty_.c_str());


  if (benchmark_mode_) {
    RCLCPP_WARN(get_logger(), "🔬 BENCHMARK MODE ENABLED: Will run %d times", max_runs_);

    if (!enable_debug) {
        // Silence everything
        RCLCPP_WARN(get_logger(), "📵 Logs SILENCED");
        
        rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);
    } else {
        // ✅ ONLY debug THIS node, not all the RCL/RMW spam!
        RCLCPP_WARN(get_logger(), "🔊 DEBUG MODE: Application logs ONLY (no RCL spam)");
        
        // Set DEFAULT to INFO (silences rcl/rmw DEBUG spam)
        rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);
        
        // ✅ Enable DEBUG for ONLY your application nodes
        rcutils_logging_set_logger_level(
            "navigate_controller", 
            RCUTILS_LOG_SEVERITY_DEBUG);
        
        rcutils_logging_set_logger_level(
            "planner_client", 
            RCUTILS_LOG_SEVERITY_INFO);
            
        rcutils_logging_set_logger_level(
            "executor", 
            RCUTILS_LOG_SEVERITY_INFO);
    }

    benchmark_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "/benchmark/run_complete", 10);
    benchmark_data_pub_ = create_publisher<std_msgs::msg::String>(
      "/benchmark/run_data", 10);
    
    RCLCPP_INFO(get_logger(), "🔬 Benchmark mode enabled");
  }

  recharge_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
  "/battery_monitor/recharge_complete",
  10,
  [this](const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "🔋 Recharge completed signal received in controller.");
      recharge_completed_ = true;
    }
  });

  RCLCPP_INFO(get_logger(), "NavigationController initialized.");
}

NavigationController::~NavigationController() = default;

// -----------------------------------------------------------------------------
// Build PDDL problem from KB
// -----------------------------------------------------------------------------
void NavigationController::build_problem_from_kb()
{
  problem_expert_->clearKnowledge();
  if (distance_map_.empty()) {
    fetch_corridor_distances();
  }

  fetch_actions();   
  fetch_waypoints();
  fetch_corridors();
  fetch_lighting_conditions();
  fetch_configurations();
  auto debug_req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  debug_req->query_type = "fetch";
  debug_req->query =
    "match "
    "  $c (constraint: $qa, constrained: $cfg) isa constraint, "
    "  has constraint-status $status, "
    "  has constraint-operator $op, "
    "  has attribute-value $val; "
    "  $qa has measure-name 'predicted-battery-level'; "
    "  $cfg has component-configuration-name $name; "
    "  (measured-attribute: $qa) isa measurement, has latest true, has measurement-value $meas; "
    "fetch $name; $op; $val; $status; $meas;";

  auto debug_fut = typedb_query_cli_->async_send_request(debug_req);
  if (debug_fut.wait_for(1s) == std::future_status::ready) {
    auto debug_resp = debug_fut.get();
    RCLCPP_ERROR(get_logger(), "🔍 ROSA Constraint Evaluation:");
    for (const auto &row : debug_resp->results) {
      std::string name, op, status;
      double val = 0, meas = 0;
      for (const auto &attr : row.attributes) {
        if (attr.name == "name") name = attr.value.string_value;
        if (attr.name == "op") op = attr.value.string_value;
        if (attr.name == "val") val = attr.value.double_value;
        if (attr.name == "status") status = attr.value.string_value;
        if (attr.name == "meas") meas = attr.value.double_value;
      }
      RCLCPP_ERROR(get_logger(), "  %s: meas=%.1f %s %.1f → status='%s'", 
        name.c_str(), meas, op.c_str(), val, status.c_str());
    }
  }
  fetch_charging_stations();
  fetch_energy_costs();
  fetch_battery_and_feasibility();
  fetch_goal();

  // ✅ Determine starting position
  std::string current_wp;
  if (first_iteration_) {
    // New run/plan - start from beginning
    current_wp = "wp_0";
  } else {
    // Mid-execution - get actual position
    current_wp = getCurrentWaypointFromFeedback();
  }
  
  problem_expert_->addPredicate(plansys2::Predicate("(at " + current_wp + ")"));

  std::ofstream out("/tmp/runtime_problem.pddl");
  out << problem_expert_->getProblem();
  out.close();
  RCLCPP_INFO(get_logger(), "Problem saved to /tmp/runtime_problem.pddl");
  
  RCLCPP_INFO(get_logger(), "cost_map_ now has %zu entries", cost_map_.size());
}

// -----------------------------------------------------------------------------
// KB fetchers
// -----------------------------------------------------------------------------
void NavigationController::fetch_actions()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $a isa Action, has action-name $name; fetch $name;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Timeout fetching actions");
    return;
  }

  auto resp = fut.get();
  if (!resp->success) {
    RCLCPP_ERROR(get_logger(), "Failed to fetch actions");
    return;
  }

  for (const auto &res : resp->results) {
    for (const auto &attr : res.attributes) {
      if (attr.label == "action-name") {
        std::string action_name = attr.value.string_value;
        
        // Add action instance
        problem_expert_->addInstance(plansys2::Instance(action_name, "action"));
        
        // Add action type predicate: move_lit_action, move_dark_action, etc.
        problem_expert_->addPredicate(plansys2::Predicate(
          "(" + action_name + "_action " + action_name + ")"
        ));
        
        RCLCPP_INFO(get_logger(), "  Added action: %s", action_name.c_str());
      }
    }
  }
  
  // ✅ Fetch action feasibility from ROSA
  fetch_action_feasibility();
}


void NavigationController::fetch_action_feasibility()
{
  // Call ROSA's selectable actions service
  auto client = this->create_client<rosa_msgs::srv::ActionQueryArray>("/rosa_kb/action/selectable");
  if (!client->wait_for_service(2s)) {
    RCLCPP_ERROR(get_logger(), "/rosa_kb/action/selectable not available");
    addAllActionsFeasible();  // fallback
    return;
  }

  auto req = std::make_shared<rosa_msgs::srv::ActionQueryArray::Request>();
  auto fut = client->async_send_request(req);

  if (fut.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Timeout fetching selectable actions");
    addAllActionsFeasible();
    return;
  }

  auto resp = fut.get();
  if (!resp->success || resp->actions.empty()) {
    RCLCPP_WARN(get_logger(), "No selectable actions returned from ROSA");
    addAllActionsFeasible();
    return;
  }

  for (const auto &action_msg : resp->actions) {
    std::string name = action_msg.name;
    problem_expert_->addPredicate(plansys2::Predicate("(action_feasible " + name + ")"));
    RCLCPP_INFO(get_logger(), "  ✅ Action feasible: %s", name.c_str());
  }
}


void NavigationController::addAllActionsFeasible()
{
  // If no status in KB, all actions are feasible
  for (const auto &action : {"move_lit", "move_dark", "recharge"}) {
    problem_expert_->addPredicate(plansys2::Predicate(
      "(action_feasible " + std::string(action) + ")"
    ));
    RCLCPP_INFO(get_logger(), "  Action feasible (default): %s", action);
  }
}

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
      corridor_pairs_.emplace_back(a, b);  // Forward
      corridor_pairs_.emplace_back(b, a);  // Backward
      
      problem_expert_->addPredicate(plansys2::Predicate("(is-corridor " + a + " " + b + ")"));
      problem_expert_->addPredicate(plansys2::Predicate("(is-corridor " + b + " " + a + ")"));  
    }
  }
}

void NavigationController::fetch_configurations()
{
  // 1) Fetch ALL configs
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $c isa component-configuration, has component-configuration-name $n; fetch $n;";
  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) return;
  auto resp = fut.get();
  if (!resp->success) return;

  std::vector<std::string> all_configs;
  for (const auto &res : resp->results)
    for (const auto &attr : res.attributes)
      if (attr.name == "n")
        all_configs.push_back(attr.value.string_value);

  // 2) Fetch FEASIBLE configs from ROSA
  auto feasible_cfgs = getFeasibleConfigsFromKB();

  RCLCPP_INFO(get_logger(), "Configuration status:");

  // 3) ✅ NEW: Add ALL configs to PDDL (not just feasible!)
  for (const auto &cfg : all_configs) {
    problem_expert_->addInstance(plansys2::Instance(cfg, "configuration"));
    problem_expert_->addPredicate(plansys2::Predicate("(config-valid " + cfg + ")"));
    
    // ✅ Only mark FEASIBLE ones with (can-use)
    if (std::find(feasible_cfgs.begin(), feasible_cfgs.end(), cfg) != feasible_cfgs.end()) {
      problem_expert_->addPredicate(plansys2::Predicate("(can-use " + cfg + ")"));
      RCLCPP_INFO(get_logger(), "  ✅ %s: FEASIBLE", cfg.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "  ❌ %s: UNFEASIBLE (no can-use)", cfg.c_str());
    }
  }

  // 5) ✅ Add corridor-specific traversal rules FOR ALL CONFIGS
  addCorridorTraversalRules(all_configs);  // ← Pass ALL, not just feasible!
}

// ✅ NEW FUNCTION: Check corridor constraints before allowing traversal
void NavigationController::addCorridorTraversalRules(
    const std::vector<std::string>& feasible_cfgs)
{
  RCLCPP_INFO(get_logger(), "🛡️ Building corridor safety rules (width + lighting)...");
  
  // Step 1: Fetch corridor widths
  auto corridor_req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  corridor_req->query_type = "fetch";
  corridor_req->query =
    "match "
    "$c (from:$w1, to:$w2) isa corridor, "
    "has corridor-width $width; "
    "$w1 has waypoint-name $a; "
    "$w2 has waypoint-name $b; "
    "fetch $a; $b; $width;";

  auto corridor_fut = typedb_query_cli_->async_send_request(corridor_req);
  if (corridor_fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "❌ Failed to fetch corridor widths");
    return;
  }
  auto corridor_resp = corridor_fut.get();

  // Step 2: Fetch width-based constraints from ROSA
  auto constraint_req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  constraint_req->query_type = "fetch";
  constraint_req->query =
    "match "
    "$cfg isa component-configuration, has component-configuration-name $name; "
    "$c (constraint: $qa, constrained: $cfg) isa constraint, "
    "  has constraint-operator $op, "
    "  has attribute-value $threshold; "
    "$qa has measure-name 'current-corridor-width'; "
    "fetch $name; $op; $threshold;";

  auto constraint_fut = typedb_query_cli_->async_send_request(constraint_req);
  if (constraint_fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "⚠️ No width constraints found - allowing all configs");
    for (const auto& [w1, w2] : corridor_pairs_) {
      for (const auto& cfg : feasible_cfgs) {
        problem_expert_->addPredicate(
          plansys2::Predicate("(can-traverse " + w1 + " " + w2 + " " + cfg + ")"));
      }
    }
    return;
  }
  auto constraint_resp = constraint_fut.get();

  // Step 3: Parse width requirements
  struct WidthConstraint {
    std::string op;
    double threshold;
  };
  std::map<std::string, WidthConstraint> config_width_reqs;

  for (const auto& row : constraint_resp->results) {
    std::string cfg_name, op;
    double threshold = 0.0;
    
    for (const auto& attr : row.attributes) {
      if (attr.name == "name") cfg_name = attr.value.string_value;
      else if (attr.name == "op") op = attr.value.string_value;
      else if (attr.name == "threshold") threshold = attr.value.double_value;
    }
    
    if (!cfg_name.empty()) {
      config_width_reqs[cfg_name] = {op, threshold};
    }
  }

  RCLCPP_INFO(get_logger(), "  Found width constraints for %zu configs", 
    config_width_reqs.size());

  // Step 4: Parse corridor widths
  std::map<std::pair<std::string, std::string>, double> corridor_widths;
  for (const auto& row : corridor_resp->results) {
    std::string w1, w2;
    double width = 0.0;
    
    for (const auto& attr : row.attributes) {
      if (attr.name == "a") w1 = attr.value.string_value;
      else if (attr.name == "b") w2 = attr.value.string_value;
      else if (attr.name == "width") width = attr.value.double_value;
    }
    
    if (!w1.empty() && !w2.empty()) {
      corridor_widths[{w1, w2}] = width;
    }
  }

  RCLCPP_INFO(get_logger(), "  Found %zu corridors with widths", corridor_widths.size());

  // ✅ NEW Step 5: Fetch lighting conditions
  auto lighting_req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  lighting_req->query_type = "fetch";
  lighting_req->query =
    "match "
    "$l (from:$w1, to:$w2) isa lighting-condition, "
    "has is-dark $dark; "
    "$w1 has waypoint-name $a; "
    "$w2 has waypoint-name $b; "
    "fetch $a; $b; $dark;";

  auto lighting_fut = typedb_query_cli_->async_send_request(lighting_req);
  
  std::map<std::pair<std::string, std::string>, bool> corridor_lighting;
  
  if (lighting_fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
    auto lighting_resp = lighting_fut.get();
    
    for (const auto& row : lighting_resp->results) {
      std::string w1, w2;
      bool is_dark = false;
      
      for (const auto& attr : row.attributes) {
        if (attr.name == "a") w1 = attr.value.string_value;
        else if (attr.name == "b") w2 = attr.value.string_value;
        else if (attr.name == "dark") is_dark = attr.value.bool_value;
      }
      
      if (!w1.empty() && !w2.empty()) {
        corridor_lighting[{w1, w2}] = is_dark;
      }
    }
    
    RCLCPP_INFO(get_logger(), "  Found lighting data for %zu corridors", 
      corridor_lighting.size());
  } else {
    RCLCPP_WARN(get_logger(), "⚠️ No lighting data found - assuming all lit");
  }

  // Step 6: Add can-traverse based on width AND lighting
  int safe_traversals = 0;
  int blocked_width = 0;
  int blocked_lighting = 0;

  for (const auto& [corridor, width] : corridor_widths) {
    const auto& [w1, w2] = corridor;
    
    // Check if corridor is dark
    bool is_dark = false;
    auto lit_it = corridor_lighting.find({w1, w2});
    if (lit_it != corridor_lighting.end()) {
      is_dark = lit_it->second;
    }
    
    for (const auto& cfg : feasible_cfgs) {
      
      // ✅ LIGHTING FILTER: Block high_speed in dark corridors
      if (is_dark && cfg == "high_speed_config") {
        blocked_lighting++;
        RCLCPP_INFO(get_logger(), "🌙 %s→%s: %s BLOCKED (dark corridor)",
          w1.c_str(), w2.c_str(), cfg.c_str());
        continue;  // Skip adding can-traverse!
      }
      
      bool allowed = true;
      
      // WIDTH FILTER: Check width constraints
      auto it = config_width_reqs.find(cfg);
      if (it != config_width_reqs.end()) {
        const auto& constraint = it->second;
        
        if (constraint.op == ">=") {
          allowed = (width >= constraint.threshold);
        } else if (constraint.op == ">") {
          allowed = (width > constraint.threshold);
        } else if (constraint.op == "<=") {
          allowed = (width <= constraint.threshold);
        } else if (constraint.op == "<") {
          allowed = (width < constraint.threshold);
        }
        
        if (allowed) {
          problem_expert_->addPredicate(plansys2::Predicate(
            "(can-traverse " + w1 + " " + w2 + " " + cfg + ")"));
          safe_traversals++;
          RCLCPP_DEBUG(get_logger(), "✅ %s→%s: %s allowed (%.2fm %s %.2fm%s)",
            w1.c_str(), w2.c_str(), cfg.c_str(), 
            width, constraint.op.c_str(), constraint.threshold,
            is_dark ? ", dark" : "");
        } else {
          blocked_width++;
          RCLCPP_INFO(get_logger(), "🚫 %s→%s: %s BLOCKED (%.2fm %s %.2fm)",
            w1.c_str(), w2.c_str(), cfg.c_str(),
            width, constraint.op.c_str(), constraint.threshold);
        }
      } else {
        // No width constraint - allow it
        problem_expert_->addPredicate(plansys2::Predicate(
          "(can-traverse " + w1 + " " + w2 + " " + cfg + ")"));
        safe_traversals++;
      }
    }
  }

  RCLCPP_INFO(get_logger(), 
    "✅ Traversal rules: %d safe, %d blocked (%d width, %d lighting)",
    safe_traversals, blocked_width, blocked_lighting);
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
  RCLCPP_INFO(get_logger(), "Computing dynamic energy costs from distances...");
  
  if (distance_map_.empty()) {
    RCLCPP_ERROR(get_logger(), "Distance map is empty! Call fetch_corridor_distances() first.");
    return;
  }
  
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match "
    "$cc (component: $c, parameter: $p) isa component-configuration, "
    "has component-configuration-name $name; "
    "$p has parameter-key 'FollowPath.max_vel_x', "
    "has parameter-value $val; "
    "fetch $name; $val;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Timeout fetching configuration speeds from TypeDB");
    return;
  }

  auto resp = fut.get();
  if (!resp->success) {
    RCLCPP_ERROR(get_logger(), "Failed to fetch configuration speeds from TypeDB");
    return;
  }

  // Build config_speeds map from TypeDB data
  std::map<std::string, double> config_speeds;
  
  for (const auto &row : resp->results) {
    std::string config_name;
    double speed = 0.0;
    
    for (const auto &attr : row.attributes) {
      if (attr.name == "name") config_name = attr.value.string_value;
      if (attr.name == "val") speed = std::stod(attr.value.string_value);
    }
    
    if (!config_name.empty() && speed > 0.0) {
      config_speeds[config_name] = speed;
      RCLCPP_INFO(get_logger(), "  Config '%s': %.2f m/s", 
        config_name.c_str(), speed);
    }
  }

  if (config_speeds.empty()) {
    RCLCPP_ERROR(get_logger(), "No configuration speeds found in TypeDB!");
    return;
  }

  cost_map_.clear();  // Clear and rebuild
  const double BASE_RATE = 1.0;
  
  // ✅ COST NOISE: ±5% variance (conservative, prevents oscillation)
  std::uniform_real_distribution<> cost_noise(0.95, 1.05);
  
  int noisy_corridors = 0;

  // Compute costs for each corridor + config combination
  for (const auto &[wp_pair, distance] : distance_map_) {
    for (const auto &[cfg, speed] : config_speeds) {
      
      double speed_factor = speed / 1.0;
      double base_cost = BASE_RATE * distance * speed_factor;
      
      // ✅ Apply 5% noise to simulate real-world uncertainty
      double noise = cost_noise(rng_);
      double cost = base_cost * noise;
      
      // Track significant deviations (>2%)
      if (std::abs(noise - 1.0) > 0.02) {
        noisy_corridors++;
        RCLCPP_DEBUG(get_logger(),
          "  🎲 %s→%s (%s): base=%.2f × noise=%.3f = %.2f",
          wp_pair.first.c_str(), wp_pair.second.c_str(), cfg.c_str(),
          base_cost, noise, cost);
      }
      
      // ✅ Store FORWARD direction
      auto key_fwd = std::make_tuple(wp_pair.first, wp_pair.second, cfg);
      cost_map_[key_fwd] = cost;
      
      // ✅ Store BACKWARD direction (same cost!)
      auto key_bwd = std::make_tuple(wp_pair.second, wp_pair.first, cfg);
      cost_map_[key_bwd] = cost;
      
      // Add to PlanSys2 (forward)
      plansys2_msgs::msg::Node node_fwd;
      node_fwd.node_type = plansys2_msgs::msg::Node::FUNCTION;
      node_fwd.name = "energy-cost";
      node_fwd.value = cost;
      
      plansys2_msgs::msg::Param p1, p2, p3;
      p1.name = wp_pair.first;
      p2.name = wp_pair.second;
      p3.name = cfg;
      node_fwd.parameters = {p1, p2, p3};
      
      plansys2::Function f_fwd(node_fwd);
      if (!problem_expert_->existFunction(f_fwd))
        problem_expert_->addFunction(f_fwd);
      
      // ✅ Add backward direction to PlanSys2
      plansys2_msgs::msg::Node node_bwd;
      node_bwd.node_type = plansys2_msgs::msg::Node::FUNCTION;
      node_bwd.name = "energy-cost";
      node_bwd.value = cost;
      
      plansys2_msgs::msg::Param p1b, p2b, p3b;
      p1b.name = wp_pair.second;  // ✅ Reversed
      p2b.name = wp_pair.first;   // ✅ Reversed
      p3b.name = cfg;
      node_bwd.parameters = {p1b, p2b, p3b};
      
      plansys2::Function f_bwd(node_bwd);
      if (!problem_expert_->existFunction(f_bwd))
        problem_expert_->addFunction(f_bwd);
    }
  }
  
  RCLCPP_INFO(get_logger(), 
    "✓ Computed %zu costs with ±5%% noise (%d corridors had >2%% variance)",
    cost_map_.size(), noisy_corridors);
}

std::vector<std::string> NavigationController::getFeasibleConfigsFromKB()
{
  std::vector<std::string> out;
  RCLCPP_INFO(get_logger(), "⏰ Querying configs NOW...");
  
  // ✅ Get ALL feasible configs with priorities
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match "
    "  $cfg isa component-configuration, "
    "  has component-configuration-name $name, "
    "  has priority $pri; "
    "  not { "  // ✅ EXCLUDE configs that have 'unfeasible' status
    "    $cfg has component-configuration-status 'unfeasible'; "
    "  }; "
    "fetch $name; $pri;";

  std::map<std::string, double> config_priorities;
  
  // ✅ Single query with longer timeout (no retry loop!)
  auto fut = typedb_query_cli_->async_send_request(req);
  
  if (fut.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "❌ Config query timeout!");
    return out;  // ← Changed from "continue" to "return"
  }
  
  auto resp = fut.get();
  
  if (!resp->success) {
    RCLCPP_ERROR(get_logger(), "❌ Config query failed!");
    return out;  // ← Changed from "continue" to "return"
  }

  for (const auto &row : resp->results) {
    std::string name;
    double pri = 999.0;
    for (const auto &attr : row.attributes) {
      if (attr.name == "name") name = attr.value.string_value;
      if (attr.name == "pri") pri = attr.value.double_value;
    }
    if (!name.empty()) {
      config_priorities[name] = pri;
    }
  }

  if (config_priorities.empty()) {
    RCLCPP_ERROR(get_logger(), "❌ No feasible configs!");
    return out;
  }

  // ✅ Sort by priority
  std::vector<std::pair<std::string, double>> sorted(
    config_priorities.begin(), config_priorities.end());
  
  std::sort(sorted.begin(), sorted.end(),
    [](const auto &a, const auto &b) { return a.second < b.second; });

  RCLCPP_INFO(get_logger(), "📋 Feasible configs:");
  
  // ✅ Return ALL feasible configs (not just the best one!)
  for (const auto &[name, pri] : sorted) {
    out.push_back(name);
    RCLCPP_INFO(get_logger(), "   %s (priority=%.1f)", name.c_str(), pri);
  }

  RCLCPP_INFO(get_logger(), "🎯 Returning %zu feasible configs", out.size());
  return out;
}


void NavigationController::fetch_corridor_distances()
{
  RCLCPP_INFO(get_logger(), "Fetching corridor distances from KB...");
  
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query =
    "match $c (from:$w1, to:$w2) isa corridor, has distance $d; "
    "$w1 has waypoint-name $a; $w2 has waypoint-name $b; "
    "fetch $a; $b; $d;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Timeout fetching corridor distances");
    return;
  }

  auto resp = fut.get();
  if (!resp->success) {
    RCLCPP_ERROR(get_logger(), "Failed to fetch corridor distances");
    return;
  }

  
  for (const auto &row : resp->results) {
    std::string a, b;
    double dist = 0.0;
    
    for (const auto &attr : row.attributes) {
      if (attr.name == "a") a = attr.value.string_value;
      if (attr.name == "b") b = attr.value.string_value;
      if (attr.name == "d") dist = attr.value.double_value;
    }
    
    if (!a.empty() && !b.empty() && dist > 0.0) {
      distance_map_[{a, b}] = dist;
      RCLCPP_INFO(get_logger(), "  Distance %s→%s: %.2f m", 
        a.c_str(), b.c_str(), dist);
    }
  }
  
  RCLCPP_INFO(get_logger(), "✓ Loaded %zu corridor distances", distance_map_.size());
}

void NavigationController::fetch_charging_stations()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = 
    "match $wp isa waypoint, has waypoint-name $name, "
    "has has-charging-station true; "
    "fetch $name;";

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) return;

  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &row : resp->results) {
    for (const auto &attr : row.attributes) {
      if (attr.name == "name") {
        std::string wp_name = attr.value.string_value;
        problem_expert_->addPredicate(
          plansys2::Predicate("(has-charging-station " + wp_name + ")"));
        RCLCPP_INFO(get_logger(), "  ⚡ Charging station at: %s", wp_name.c_str());
      }
    }
  }
}

void NavigationController::fetch_battery_and_feasibility()
{
  // ✅ CORRECT WAY - with node type!
  plansys2_msgs::msg::Node batt_node;
  batt_node.node_type = plansys2_msgs::msg::Node::FUNCTION;
  batt_node.name = "battery-level";
  batt_node.value = battery_level_;
  
  plansys2::Function batt(batt_node);
  
  if (!problem_expert_->existFunction(batt)) {
    problem_expert_->addFunction(batt);
  } else {
    problem_expert_->updateFunction(batt);
  }

  // ✅ REACTIVE MODE: Allow ALL configs in PDDL
  // Battery constraints are enforced by ROSA at runtime, not planner!
  std::vector<std::string> all_configs = {
    "high_speed_config",
    "low_speed_config", 
    "degraded_speed_config"
  };
  
  for (const auto &[w1, w2] : corridor_pairs_) {
    for (const auto &cfg : all_configs) {  // ← Use ALL configs, not just feasible!
      problem_expert_->addPredicate(plansys2::Predicate(
        "(has-enough-battery " + w1 + " " + w2 + " " + cfg + ")"));
    }
  }
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
  battery_level_ = pct;  // Store locally
  RCLCPP_DEBUG(get_logger(), "Battery updated: %.2f%%", battery_level_);
}


// -----------------------------------------------------------------------------
// Reactive check
// -----------------------------------------------------------------------------


void NavigationController::triggerGoalModification()
{
  RCLCPP_ERROR(get_logger(), "🔄 [TASK ADAPTATION] Reducing goal scope!");
  
  // Find current position
  auto feedback = executor_client_->getFeedBack();
  std::string current_wp = "wp_0";
  
  for (const auto &ae : feedback.action_execution_status) {
    if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
      auto [from, to, cfg] = parse_action(ae.action);
      if (!to.empty()) {
        current_wp = to;
      }
    }
  }
  
  // Calculate reachable goal
  std::string new_goal = findNearestReachableGoal(current_wp);
  
  RCLCPP_WARN(get_logger(), "🎯 New goal: %s (reduced from wp_10)", new_goal.c_str());
  
  // ✅ Update goal in Problem Expert
  problem_expert_->setGoal(plansys2::Goal("(and (at " + new_goal + "))"));
  
  // Also update KB for consistency
  updateGoalInKB(new_goal);
  
  // ✅ Get new plan with updated problem
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);
  
  if (!plan.has_value()) {
    RCLCPP_ERROR(get_logger(), "❌ No plan with reduced goal!");
    finish_controlling();
    return;
  }
  
  RCLCPP_INFO(get_logger(), "✅ New plan: %zu actions to %s", 
    plan->items.size(), new_goal.c_str());
  
  // ✅ Send new plan (triggers seamless replan internally!)
  executor_client_->start_plan_execution(plan.value());
  
  RCLCPP_INFO(get_logger(), "🚀 SEAMLESS REPLAN! Robot continues WITHOUT stopping!");
}

std::string NavigationController::findNearestReachableGoal(const std::string &current_wp)
{
  // Extract waypoint number: "wp_5" → 5
  int current_num = std::stoi(current_wp.substr(3));
  
  // Calculate available battery (minus safety margin)
  double available_battery = battery_level_ - safety_margin_;
  
  RCLCPP_INFO(get_logger(), 
    "🧮 Calculating reachable goal from %s with %.2f%% available battery",
    current_wp.c_str(), available_battery);
  
  // ✅ Try each waypoint from current to wp_10, check if reachable
  std::string best_reachable_goal;
  
  for (int target_num = 10; target_num > current_num; target_num--) {
    std::string candidate_goal = "wp_" + std::to_string(target_num);
    
    // Calculate actual cost from current position to candidate goal
    double total_cost = calculatePathCost(current_wp, candidate_goal);
    
    if (total_cost < 0) {
      // Cost not found in map, skip
      continue;
    }
    
    RCLCPP_DEBUG(get_logger(), 
      "  Checking %s: cost=%.2f, available=%.2f",
      candidate_goal.c_str(), total_cost, available_battery);
    
    // Check if we have enough battery to reach this goal
    if (total_cost <= available_battery) {
      best_reachable_goal = candidate_goal;
      RCLCPP_INFO(get_logger(), 
        "✅ Found reachable goal: %s (cost=%.2f%%, available=%.2f%%)",
        candidate_goal.c_str(), total_cost, available_battery);
      break;  // Found furthest reachable goal
    }
  }
  
  if (best_reachable_goal.empty()) {
    RCLCPP_ERROR(get_logger(), "❌ No reachable goal found from %s!", current_wp.c_str());
  }
  
  return best_reachable_goal;
}
std::string NavigationController::getNearestWaypointFromAMCL()
{
  if (!latest_amcl_pose_received_) {
    RCLCPP_WARN(get_logger(), "⚠️ No AMCL pose yet, fallback to wp_0");
    return "wp_0";
  }

  geometry_msgs::msg::Pose robot_pose = amcl_pose_.pose.pose;
  std::string nearest_wp = "wp_0";
  double min_dist = std::numeric_limits<double>::max();

  // ────────────────────────────────────────────────
  //  Real waypoint coordinates (copied from YAML)
  //  Format: { waypoint_name : {x, y} }
  // ────────────────────────────────────────────────
  std::map<std::string, std::pair<double, double>> waypoints = {
    {"wp_0",  {0.0, 0.0}},
    {"wp_1",  {-4.7, -2.5}},
    {"wp_2",  {-7.7, 0.0}},
    {"wp_3",  {-8.0, 5.5}},
    {"wp_4",  {-1.53841, 4.0}},
    {"wp_5",  {-1.0, 0.0}},
    {"wp_6",  {-8.0, -1.5}},
    {"wp_7",  {-8.0, 3.5}},
    {"wp_8",  {0.0, 6.0}},
    {"wp_9",  {-6.5, -1.5}},
    {"wp_10", {-6.0, 4.5}}
  };
  // ────────────────────────────────────────────────

  for (const auto & [name, coords] : waypoints) {
    double dist = std::hypot(robot_pose.position.x - coords.first,
                             robot_pose.position.y - coords.second);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_wp = name;
    }
  }

  if (min_dist == std::numeric_limits<double>::max()) {
    RCLCPP_ERROR(get_logger(), "❌ No waypoint distances computed! Fallback to wp_0.");
    return "wp_0";
  }

  RCLCPP_INFO(get_logger(), "🧭 AMCL Pose: x=%.2f, y=%.2f", 
              robot_pose.position.x, robot_pose.position.y);
  RCLCPP_INFO(get_logger(), "📍 Nearest waypoint: %s (%.2f m)", 
              nearest_wp.c_str(), min_dist);

  return nearest_wp;
}
std::string NavigationController::findNearestCharger(const std::string &from_wp)
{
  // Query all charging stations from KB
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = 
    "match $wp isa waypoint, has waypoint-name $name, "
    "has has-charging-station true; fetch $name;";
  
  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Failed to fetch chargers, fallback to current position");
    return from_wp;
  }
  
  auto resp = fut.get();
  if (!resp->success || resp->results.empty()) {
    RCLCPP_WARN(get_logger(), "No chargers found in KB, fallback to current position");
    return from_wp;
  }
  
  // Collect all chargers
  std::vector<std::string> chargers;
  for (const auto &row : resp->results) {
    for (const auto &attr : row.attributes) {
      if (attr.name == "name") {
        chargers.push_back(attr.value.string_value);
      }
    }
  }
  
  // If current position HAS a charger, use it (recharge in place)
  if (std::find(chargers.begin(), chargers.end(), from_wp) != chargers.end()) {
    RCLCPP_INFO(get_logger(), 
      "✅ Already at charger %s, will recharge in place", from_wp.c_str());
    return from_wp;
  }
  
  // Find nearest charger by waypoint number
  int from_num = std::stoi(from_wp.substr(3));
  std::string nearest = chargers[0];
  int min_distance = 999;
  
  for (const auto &charger : chargers) {
    int charger_num = std::stoi(charger.substr(3));
    int dist = std::abs(charger_num - from_num);
    
    if (dist < min_distance) {
      min_distance = dist;
      nearest = charger;
    }
  }
  
  RCLCPP_INFO(get_logger(), 
    "🎯 Nearest charger from %s: %s (%d waypoints away)",
    from_wp.c_str(), nearest.c_str(), min_distance);
  
  return nearest;
}


double NavigationController::calculatePathCost(
  const std::string &from_wp, 
  const std::string &to_wp)
{
  // Extract waypoint numbers
  int from_num = std::stoi(from_wp.substr(3));
  int to_num = std::stoi(to_wp.substr(3));
  
  if (from_num >= to_num) {
    return -1.0;  // Invalid path
  }
  
  double total_cost = 0.0;
  
  // Sum costs of all corridors from 'from' to 'to'
  for (int i = from_num; i < to_num; i++) {
    std::string start = "wp_" + std::to_string(i);
    std::string end = "wp_" + std::to_string(i + 1);
    
    // Use low_speed_config since we're in task adaptation phase
    auto key = std::make_tuple(start, end, "low_speed_config");
    
    if (cost_map_.count(key)) {
      total_cost += cost_map_[key];
    } else {
      RCLCPP_WARN(get_logger(), 
        "Cost not found for %s→%s (low_speed)", start.c_str(), end.c_str());
      return -1.0;  // Cost not available
    }
  }
  
  return total_cost;
}

void NavigationController::updateGoalInKB(const std::string &new_goal)
{
  // Delete old goal
  auto del_req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  del_req->query_type = "delete";
  del_req->query = "match $g isa goal; delete $g isa goal;";
  
  auto del_fut = typedb_query_cli_->async_send_request(del_req);
  
  if (del_fut.wait_for(500ms) == std::future_status::ready) {
    // Insert new goal
    auto ins_req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    ins_req->query_type = "insert";
    ins_req->query = 
      "insert $g isa goal, has goal-name '" + new_goal + "';";
    
    typedb_query_cli_->async_send_request(ins_req);
    
    RCLCPP_INFO(get_logger(), "✅ Goal updated in KB: %s", new_goal.c_str());
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
  
  std::string first_config;  // ✅ NEW: Extract first action's config
  
  for (size_t i = 0; i < plan->items.size(); i++) {
    const auto &item = plan->items[i];
    RCLCPP_INFO(get_logger(), "[%zu] Action: '%s'", i, item.action.c_str());
    
    auto [from, to, cfg] = parse_action(item.action);
    
    // ✅ NEW: Capture first config
    if (i == 0 && !cfg.empty()) {
      first_config = cfg;
    }
    
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
  
  // ✅ FIX: Update current_config_ and tell ROSA to use it!
  if (!current_plan_actions_.empty() && !first_config.empty()) {
    current_config_ = first_config;
    RCLCPP_INFO(get_logger(), 
      "📌 Plan uses configuration: '%s'", current_config_.c_str());

    
    // ✅ NEW: Trigger ROSA to apply the configuration
    RCLCPP_INFO(get_logger(), "🔄 Triggering ROSA to apply '%s'", first_config.c_str());
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "insert_monitoring_data";
    rosa_event_pub_->publish(std::move(msg));
    
    // Wait for ROSA to reconfigure
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
  RCLCPP_INFO(get_logger(), "======================\n");

  executor_client_->start_plan_execution(plan.value());
}

void NavigationController::deleteMapFromKB() {
  RCLCPP_INFO(get_logger(), "🗑️ Deleting old map from KB...");
  
  // Delete lighting
  auto del_lighting = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  del_lighting->query_type = "delete";
  del_lighting->query = "match $l isa lighting-condition; delete $l isa lighting-condition;";
  auto fut1 = typedb_query_cli_->async_send_request(del_lighting);
  fut1.wait_for(std::chrono::seconds(5));
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // Delete corridors
  auto del_corridors = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  del_corridors->query_type = "delete";
  del_corridors->query = "match $c isa corridor; delete $c isa corridor;";
  auto fut2 = typedb_query_cli_->async_send_request(del_corridors);
  fut2.wait_for(std::chrono::seconds(5));
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // Delete waypoints
  auto del_waypoints = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  del_waypoints->query_type = "delete";
  del_waypoints->query = "match $w isa waypoint; delete $w isa waypoint;";
  auto fut3 = typedb_query_cli_->async_send_request(del_waypoints);
  fut3.wait_for(std::chrono::seconds(5));
  
  RCLCPP_INFO(get_logger(), "✅ Map deleted from KB");
}


void NavigationController::generateAndInjectMap()
{
  // ═══════════════════════════════════════════════════════════════════
  // LOOP-BASED GENERATION (replaces recursion)
  // ═══════════════════════════════════════════════════════════════════
  const int MAX_RETRIES = 5000;
  int retry_count = 0;
  bool map_generated = false;
  
  // Variables that need to persist for injection
  ScenarioDifficulty difficulty;
  int num_waypoints = 0;
  std::vector<Edge> final_edges;
  std::vector<int> charger_nodes;
  
  while (!map_generated && retry_count < MAX_RETRIES) {
    map_seed_++;
    rng_.seed(map_seed_);
    
    // ═══════════════════════════════════════════════════════════════════
    // STEP 0: Select difficulty parameters
    // ═══════════════════════════════════════════════════════════════════
    
    if (scenario_difficulty_ == "easy") {
      difficulty = {
        .nodes_skip = 0.00,
        .edges_remove = 0.30,
        .min_chargers = 6,
        .max_chargers = 8,
        .name = "EASY (Dense, many chargers)"
      };
    } 
    else if (scenario_difficulty_ == "medium") {
      difficulty = {
        .nodes_skip = 0.00,
        .edges_remove = 0.35,
        .min_chargers = 4,
        .max_chargers = 5,
        .name = "MEDIUM (Moderate connectivity)"
      };
    }
    else if (scenario_difficulty_ == "hard") {
      difficulty = {
        .nodes_skip = 0.00,
        .edges_remove = 0.40,
        .min_chargers = 3,
        .max_chargers = 4,
        .name = "HARD (Sparse, few chargers)"
      };
    }
    else {
      RCLCPP_ERROR(get_logger(), 
        "❌ Unknown difficulty '%s', using EASY", scenario_difficulty_.c_str());
      difficulty = {
        .nodes_skip = 0.10,
        .edges_remove = 0.10,
        .min_chargers = 6,
        .max_chargers = 8,
        .name = "EASY (fallback)"
      };
    }
    
    if (retry_count == 0) {
      RCLCPP_INFO(get_logger(), 
        "🎲 Generating %s map (seed: %d)", difficulty.name.c_str(), map_seed_);
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // STEP 1: Generate Grid Layout
    // ═══════════════════════════════════════════════════════════════════
    
    const int GRID_SIZE = 8;
    const double SPACING = 5.0;
    
    std::vector<std::pair<int, int>> grid_positions;
    for (int row = 0; row < GRID_SIZE; row++) {
      for (int col = 0; col < GRID_SIZE; col++) {
        grid_positions.push_back({row, col});
      }
    }
    
    int total_nodes = static_cast<int>(grid_positions.size());
    int nodes_to_skip = static_cast<int>(total_nodes * difficulty.nodes_skip);
    
    std::vector<int> skip_indices;
    for (int i = 0; i < nodes_to_skip; i++) {
      int skip_idx = std::uniform_int_distribution<>(0, total_nodes - 1)(rng_);
      skip_indices.push_back(skip_idx);
    }
    
    std::vector<std::pair<int, int>> active_positions;
    std::map<std::pair<int, int>, int> pos_to_wp_id;
    
    int wp_id = 0;
    for (size_t i = 0; i < grid_positions.size(); i++) {
      if (std::find(skip_indices.begin(), skip_indices.end(), static_cast<int>(i)) == skip_indices.end()) {
        active_positions.push_back(grid_positions[i]);
        pos_to_wp_id[grid_positions[i]] = wp_id;
        wp_id++;
      }
    }
    
    num_waypoints = static_cast<int>(active_positions.size());
    
    if (retry_count == 0) {
      RCLCPP_INFO(get_logger(), 
        "  Grid: %dx%d, Active nodes: %d (skipped %d)", 
        GRID_SIZE, GRID_SIZE, num_waypoints, nodes_to_skip);
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // STEP 2: Generate Edges
    // ═══════════════════════════════════════════════════════════════════
    
    std::vector<Edge> all_edges;
    
    for (size_t i = 0; i < active_positions.size(); i++) {
      auto [row_i, col_i] = active_positions[i];
      
      for (size_t j = i + 1; j < active_positions.size(); j++) {
        auto [row_j, col_j] = active_positions[j];
        
        bool horizontal = (row_i == row_j && std::abs(col_i - col_j) == 1);
        bool vertical = (col_i == col_j && std::abs(row_i - row_j) == 1);
        
        if (horizontal || vertical) {
          double dist = SPACING;
          all_edges.push_back({static_cast<int>(i), static_cast<int>(j), dist});
        }
      }
    }
    
    if (retry_count == 0) {
      RCLCPP_INFO(get_logger(), "  Generated %zu potential edges", all_edges.size());
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // STEP 3: Remove Edges Based on Difficulty
    // ═══════════════════════════════════════════════════════════════════
    
    std::shuffle(all_edges.begin(), all_edges.end(), rng_);
    
    int edges_to_remove = static_cast<int>(all_edges.size() * difficulty.edges_remove);
    final_edges = std::vector<Edge>(
      all_edges.begin() + edges_to_remove, 
      all_edges.end()
    );
    
    if (retry_count == 0) {
      RCLCPP_INFO(get_logger(), 
        "  Removed %d edges (%.0f%%), keeping %zu", 
        edges_to_remove, difficulty.edges_remove * 100, final_edges.size());
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // STEP 4: Check Connectivity
    // ═══════════════════════════════════════════════════════════════════
    
    if (isGraphConnected(num_waypoints, final_edges)) {
      map_generated = true;
      RCLCPP_INFO(get_logger(), "✅ Graph is connected!");
      
      // ═══════════════════════════════════════════════════════════════════
      // STEP 5: Place Chargers Based on Difficulty
      // ═══════════════════════════════════════════════════════════════════
      
      int num_chargers = std::uniform_int_distribution<>(
        difficulty.min_chargers, 
        difficulty.max_chargers
      )(rng_);
      
      charger_nodes.clear();
      
      if (scenario_difficulty_ == "hard") {
        int max_charger_wp = std::max(3, num_waypoints / 3);
        for (int i = 0; i < num_chargers; i++) {
          int charger_wp = std::uniform_int_distribution<>(0, max_charger_wp)(rng_);
          charger_nodes.push_back(charger_wp);
        }
        RCLCPP_INFO(get_logger(), 
          "  Placed %d chargers (CLUSTERED for hard mode)", num_chargers);
      } 
      else {
        int step = num_waypoints / (num_chargers + 1);
        for (int i = 0; i < num_chargers; i++) {
          int charger_wp = (i + 1) * step + 
            std::uniform_int_distribution<>(-step/3, step/3)(rng_);
          charger_wp = std::max(0, std::min(num_waypoints - 1, charger_wp));
          charger_nodes.push_back(charger_wp);
        }
        RCLCPP_INFO(get_logger(), 
          "  Placed %d chargers (DISTRIBUTED)", num_chargers);
      }
      
    } else {
      retry_count++;
      
      // Log progress every 500 attempts
      if (retry_count % 500 == 0) {
        RCLCPP_WARN(get_logger(), 
          "⏳ Still searching for connected graph... (attempt %d/%d)", 
          retry_count, MAX_RETRIES);
      }
    }
  }
  
  // ═══════════════════════════════════════════════════════════════════
  // Check if we succeeded or failed
  // ═══════════════════════════════════════════════════════════════════
  
  if (!map_generated) {
    RCLCPP_ERROR(get_logger(), 
      "❌ Failed to generate connected map after %d attempts!", MAX_RETRIES);
    RCLCPP_ERROR(get_logger(), 
      "   Last seed tried: %d (difficulty: %s)", 
      map_seed_, difficulty.name.c_str());
    
    if (benchmark_mode_) {
      current_run_data_.success = false;
      current_run_data_.failure_reason = "map_generation_failed";
    }
    
    finish_controlling();
    return;
  }
  
  // ═══════════════════════════════════════════════════════════════════
  // STEP 6: Inject into TypeDB
  // ═══════════════════════════════════════════════════════════════════
  
  std::stringstream query;
  query << "insert ";
  
  // Add waypoints
  for (int i = 0; i < num_waypoints; i++) {
    bool has_charger = std::find(charger_nodes.begin(), charger_nodes.end(), i) 
                        != charger_nodes.end();
    
    query << "$w" << i << " isa waypoint, "
          << "has waypoint-name 'wp_" << i << "'";
    
    if (has_charger) {
      query << ", has has-charging-station true";
    }
    
    query << "; ";
  }
  
  // Add corridors with random widths
  std::uniform_real_distribution<> width_dist(0.0, 1.0);
  int corr_idx = 0;

  // ✅ NEW: Decide which corridors are dark (30% of them)
  std::vector<bool> is_dark_corridor;
  for (size_t i = 0; i < final_edges.size(); i++) {
    is_dark_corridor.push_back(width_dist(rng_) < 0.3);  // 30% dark
  }

  for (size_t edge_idx = 0; edge_idx < final_edges.size(); edge_idx++) {
    const auto& edge = final_edges[edge_idx];
    
    double rand_val = width_dist(rng_);
    double width, max_speed;
    
    if (rand_val < 0.25) {
      width = 0.8 + width_dist(rng_) * 0.2;
      max_speed = 1.0;
    } else if (rand_val < 0.5) {
      width = 1.4 + width_dist(rng_) * 0.2;
      max_speed = 1.5;
    } else {
      width = 1.6 + width_dist(rng_) * 0.4;
      max_speed = 2.0;
    }
    
    const char* obstacles[] = {"low", "medium", "high"};
    const char* obstacle = obstacles[static_cast<int>(width_dist(rng_) * 3) % 3];
    
    // ✅ Check if this corridor is dark
    bool is_dark = is_dark_corridor[edge_idx];
    
    // Forward corridor
    query << "$c" << corr_idx << " (from: $w" << edge.from_wp 
          << ", to: $w" << edge.to_wp << ") isa corridor, "
          << "has distance " << std::fixed << std::setprecision(1) << edge.distance << ", "
          << "has corridor-width " << std::setprecision(1) << width << ", "
          << "has max-safe-speed " << max_speed << ", "
          << "has obstacle-density '" << obstacle << "'; ";
    
    // ✅ Forward lighting (conditional)
    if (is_dark) {
      query << "$l" << corr_idx << " (from: $w" << edge.from_wp 
            << ", to: $w" << edge.to_wp << ") isa lighting-condition, "
            << "has is-lit false, has is-dark true; ";
    } else {
      query << "$l" << corr_idx << " (from: $w" << edge.from_wp 
            << ", to: $w" << edge.to_wp << ") isa lighting-condition, "
            << "has is-lit true, has is-dark false; ";
    }
    corr_idx++;
    
    // Backward corridor
    query << "$c" << corr_idx << " (from: $w" << edge.to_wp 
          << ", to: $w" << edge.from_wp << ") isa corridor, "
          << "has distance " << std::setprecision(1) << edge.distance << ", "
          << "has corridor-width " << std::setprecision(1) << width << ", "
          << "has max-safe-speed " << max_speed << ", "
          << "has obstacle-density '" << obstacle << "'; ";
    
    // ✅ Backward lighting (same as forward!)
    if (is_dark) {
      query << "$l" << corr_idx << " (from: $w" << edge.to_wp 
            << ", to: $w" << edge.from_wp << ") isa lighting-condition, "
            << "has is-lit false, has is-dark true; ";
    } else {
      query << "$l" << corr_idx << " (from: $w" << edge.to_wp 
            << ", to: $w" << edge.from_wp << ") isa lighting-condition, "
            << "has is-lit true, has is-dark false; ";
    }
    corr_idx++;
  }
  
  // Insert
  auto ins_req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  ins_req->query_type = "insert";
  ins_req->query = query.str();
  
  auto ins_fut = typedb_query_cli_->async_send_request(ins_req);
  if (ins_fut.wait_for(10s) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), 
      "✅ %s map injected: %d waypoints, %zu corridors, %d chargers",
      difficulty.name.c_str(), num_waypoints, final_edges.size(), charger_nodes.size());
  } else {
    RCLCPP_ERROR(get_logger(), "❌ Map injection TIMEOUT!");
  }
}

// ═══════════════════════════════════════════════════════════════════════
// Helper: Check if graph is connected (BFS)
// ═══════════════════════════════════════════════════════════════════════

bool NavigationController::isGraphConnected(
    int num_nodes, 
    const std::vector<Edge>& edges)
{
  // Build adjacency list
  std::map<int, std::vector<int>> adj;
  for (const auto& edge : edges) {
    adj[edge.from_wp].push_back(edge.to_wp);
    adj[edge.to_wp].push_back(edge.from_wp);
  }
  
  // BFS from node 0
  std::set<int> visited;
  std::queue<int> q;
  q.push(0);
  visited.insert(0);
  
  while (!q.empty()) {
    int node = q.front();
    q.pop();
    
    for (int neighbor : adj[node]) {
      if (visited.find(neighbor) == visited.end()) {
        visited.insert(neighbor);
        q.push(neighbor);
      }
    }
  }
  
  return visited.size() == static_cast<size_t>(num_nodes);  // ✅ FIXED
}
void NavigationController::initializeCorridorWidthInKB(double width)
{
  auto diag_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  diag_msg->header.stamp = this->now();
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_controller";
  status.message = "attribute measurement";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "current-corridor-width";
  kv.value = std::to_string(width);
  status.values.push_back(kv);
  
  diag_msg->status.push_back(status);
  diagnostics_pub_->publish(std::move(diag_msg));
  
  RCLCPP_INFO(get_logger(), 
    "🛡️ Initialized corridor width to %.2fm for new run", width);
}

void NavigationController::finish_controlling()
{
  current_run_++;
  
  if (benchmark_mode_) {
    // ✅ PUBLISH BENCHMARK DATA FOR COMPLETED RUN
    if (current_run_ > 1) {
      publishBenchmarkData();
    }
    
    // Signal run complete to Python monitor
    auto msg = std_msgs::msg::Int32();
    msg.data = current_run_;
    benchmark_pub_->publish(msg);
    
    RCLCPP_WARN(get_logger(), 
      "✅ Run %d/%d complete. Restarting...", 
      current_run_, max_runs_);
    
    if (current_run_ >= max_runs_) {
      RCLCPP_INFO(get_logger(), 
        "🏁 BENCHMARK COMPLETE: %d runs finished!", max_runs_);
      
      step_timer_->cancel();
      executor_client_->cancel_plan_execution();
      return;
    }
    
    stop_monitoring();
    executor_client_->cancel_plan_execution();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ✅ Clear state
    // problem_expert_->clearKnowledge();
    deleteMapFromKB();
    generateAndInjectMap();
    fetch_corridor_distances();
    // fetch_energy_costs();
    
    RCLCPP_INFO(get_logger(), "⏳ Waiting for map regeneration...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    start_monitoring();
   

    // Reset state
    first_iteration_ = true;
    current_goal_ = "navigation";
    recharge_completed_ = false;
    pending_recharge_ = false;
    last_action.clear();
    current_plan_actions_.clear();
    
    // ✅ Start with high_speed_config at 100% battery
    current_config_ = "high_speed_config";
    RCLCPP_INFO(get_logger(), "⚙️ Reset config to: %s", current_config_.c_str());
    
    // ✅ Reset battery to 100% for new run
    battery_level_ = 100.0;
    
    // ✅ Publish reset to ROSA so it re-evaluates from scratch
    RCLCPP_INFO(get_logger(), "🔋 Resetting battery to 100%% for new run");
    publishBatteryMeasurement(100.0, "battery_reset");  // ✅ Use this!
    
    // ✅ TRIGGER ROSA
    auto rosa_msg = std::make_unique<std_msgs::msg::String>();
    rosa_msg->data = "insert_monitoring_data";
    rosa_event_pub_->publish(std::move(rosa_msg));
    
    initializeCorridorWidthInKB(2.0);
    
    // Give ROSA time to process reset
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    
    // ✅ INITIALIZE NEW RUN DATA TRACKING
    initializeBenchmarkRun();
    
    RCLCPP_INFO(get_logger(), "🔄 Starting run %d/%d with fresh configuration", 
      current_run_ + 1, max_runs_);
    
    return;
  }
  
  // Normal mode: just stop
  step_timer_->cancel();
  executor_client_->cancel_plan_execution();
}



void NavigationController::stop_monitoring() {
  if (step_timer_) {
    step_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "⏸️  Paused step timer for map regeneration");
}

void NavigationController::start_monitoring() {
  // Recreate the timer
  step_timer_ = this->create_wall_timer(
    1s, 
    std::bind(&NavigationController::step, this), 
    step_timer_cb_group_
  );
  RCLCPP_INFO(get_logger(), "▶️  Resumed step timer");
}
// -----------------------------------------------------------------------------
// Helper stubs for goal switching loop
// -----------------------------------------------------------------------------


std::vector<std::string> NavigationController::getFeasibleActionsFromRosa()
{
  std::vector<std::string> feasible;
  auto client = this->create_client<rosa_msgs::srv::ActionQueryArray>("/rosa_kb/action/selectable");
  if (!client->wait_for_service(1s)) {
    RCLCPP_WARN(get_logger(), "/rosa_kb/action/selectable unavailable, assuming all feasible");
    return {"move_lit", "move_dark", "recharge"};
  }

  auto req = std::make_shared<rosa_msgs::srv::ActionQueryArray::Request>();
  auto fut = client->async_send_request(req);
  if (fut.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Timeout fetching selectable actions");
    return {"move_lit", "move_dark", "recharge"};
  }

  auto resp = fut.get();
  if (!resp->success || resp->actions.empty()) {
    RCLCPP_WARN(get_logger(), "No actions returned from ROSA");
    return {"move_lit", "move_dark", "recharge"};
  }

  for (const auto& a : resp->actions)
    feasible.push_back(a.name);

  return feasible;
}

bool NavigationController::isActionFeasible(
  const std::vector<std::string>& feasible_actions,
  const std::string& action_name)
{
  return std::find(feasible_actions.begin(), feasible_actions.end(), action_name)
         != feasible_actions.end();
}

std::string NavigationController::getCurrentWaypointFromFeedback()
{
  auto feedback = executor_client_->getFeedBack();
  std::string current_wp = "wp_0";  // fallback
  
  RCLCPP_INFO(get_logger(), "🔍 Getting current waypoint from %zu actions", 
    feedback.action_execution_status.size());
  
  // ✅ PASS 1: Find EXECUTING action (highest priority)
  for (const auto &ae : feedback.action_execution_status) {
    if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
      
      // ✅ Handle move actions using arguments array!
      if (ae.action == "move_lit" || ae.action == "move_dark") {
        // Arguments format: [action_name, from_wp, to_wp, config]
        // Example: ["move_lit", "wp_7", "wp_8", "high_speed_config"]
        
        if (ae.arguments.size() >= 3) {
          std::string from = ae.arguments[1];  
          std::string to = ae.arguments[2];    
          
          RCLCPP_INFO(get_logger(), 
            "✅ Found EXECUTING %s: robot at %s (moving to %s)", 
            ae.action.c_str(), from.c_str(), to.c_str());
          
          return from;  
        }
      }
      
      // ✅ Handle recharge using arguments array
      else if (ae.action == "recharge") {
        // Arguments format: ["recharge", "wp_3"]
        if (ae.arguments.size() >= 2) {
          std::string wp = ae.arguments[1];  // wp_3
          
          RCLCPP_INFO(get_logger(), 
            "✅ Found EXECUTING recharge at: %s", wp.c_str());
          
          return wp;
        }
      }
    }
  }
  
  // ✅ PASS 2: No executing action, find last SUCCEEDED action
  std::string last_completed = "wp_0";
  
  for (const auto &ae : feedback.action_execution_status) {
    if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
      
      // Handle completed move actions
      if (ae.action == "move_lit" || ae.action == "move_dark") {
        if (ae.arguments.size() >= 3) {
          last_completed = ae.arguments[2];  // Robot reached 'to' waypoint
          RCLCPP_DEBUG(get_logger(), 
            "Found SUCCEEDED move to: %s", last_completed.c_str());
        }
      }
      
      // Handle completed recharge
      else if (ae.action == "recharge") {
        if (ae.arguments.size() >= 2) {
          last_completed = ae.arguments[1];
          RCLCPP_DEBUG(get_logger(), 
            "Found SUCCEEDED recharge at: %s", last_completed.c_str());
        }
      }
    }
  }
  
  current_wp = last_completed;
  RCLCPP_INFO(get_logger(), 
    "No executing action. Using last completed waypoint: %s", current_wp.c_str());
  
  return current_wp;
}

void NavigationController::updateCurrentWaypointInKB(const std::string &wp)
{
  // Small buffer so ROSA/TypeDB can finish any previous TXNs
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  if (!typedb_query_cli_) {
    RCLCPP_WARN(get_logger(), "Query client not ready; skip updating current waypoint to %s", wp.c_str());
    return;
  }
  if (!typedb_query_cli_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "/rosa_kb/query unavailable; skip updating waypoint %s", wp.c_str());
    return;
  }

  // Best-effort, schema-agnostic event insert (safe no-op if rule absent).
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "insert";
  req->query =
    "insert $e isa navigation-event, "
    "has event-type 'set-current-waypoint', "
    "has event-value '" + wp + "';";

  try {
    auto fut = typedb_query_cli_->async_send_request(req);
    if (fut.wait_for(std::chrono::milliseconds(800)) == std::future_status::ready) {
      auto resp = fut.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "Current waypoint updated to %s", wp.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to update waypoint %s (no error string exposed by service)",
            wp.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(), "Timeout while updating waypoint to %s", wp.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Exception updating waypoint %s: %s", wp.c_str(), e.what());
  }

  // Small delay so subsequent planner calls don’t collide with KB writes
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
}



double NavigationController::queryCorridorWidth(
    const std::string& from_wp, 
    const std::string& to_wp)
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = 
    "match "
    "$c (from: $w1, to: $w2) isa corridor, has corridor-width $width; "
    "$w1 has waypoint-name '" + from_wp + "'; "
    "$w2 has waypoint-name '" + to_wp + "'; "
    "fetch $width;";
  
  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(500ms) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Timeout querying corridor %s→%s width", 
      from_wp.c_str(), to_wp.c_str());
    return 2.0;  // Default safe
  }
  
  auto resp = fut.get();
  if (!resp->success || resp->results.empty()) {
    RCLCPP_WARN(get_logger(), "No width data for corridor %s→%s", 
      from_wp.c_str(), to_wp.c_str());
    return 2.0;
  }
  
  for (const auto &row : resp->results) {
    for (const auto &attr : row.attributes) {
      if (attr.name == "width") {
        double width = attr.value.double_value;
        RCLCPP_INFO(get_logger(), "🛡️ Corridor %s→%s: width=%.2fm", 
          from_wp.c_str(), to_wp.c_str(), width);
        return width;
      }
    }
  }
  
  return 2.0;
}

bool NavigationController::queryCorridorLighting(
    const std::string& from_wp, 
    const std::string& to_wp)
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = 
    "match "
    "$l (from: $w1, to: $w2) isa lighting-condition, has is-dark $dark; "
    "$w1 has waypoint-name '" + from_wp + "'; "
    "$w2 has waypoint-name '" + to_wp + "'; "
    "$l has is-dark $dark; "
    "fetch $dark;";
  
  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(500ms) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Timeout querying corridor %s→%s lighting", 
      from_wp.c_str(), to_wp.c_str());
    return false;  // Default: assume lit
  }
  
  auto resp = fut.get();
  if (!resp->success || resp->results.empty()) {
    RCLCPP_WARN(get_logger(), "No lighting data for corridor %s→%s", 
      from_wp.c_str(), to_wp.c_str());
    return false;  // Default: assume lit
  }
  
  for (const auto &row : resp->results) {
    for (const auto &attr : row.attributes) {
      if (attr.name == "dark") {
        bool is_dark = attr.value.bool_value;
        RCLCPP_INFO(get_logger(), "🌙 Corridor %s→%s: %s", 
          from_wp.c_str(), to_wp.c_str(), is_dark ? "DARK" : "LIT");
        return is_dark;
      }
    }
  }
  
  return false;  // Default: assume lit
}

void NavigationController::publishCorridorSafety(
    const std::string& from_wp,
    const std::string& to_wp)
{
  double width = queryCorridorWidth(from_wp, to_wp);
  bool is_dark = queryCorridorLighting(from_wp, to_wp);  // ✅ ADD THIS
  
  // Publish to ROSA via /diagnostics
  auto diag_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  diag_msg->header.stamp = this->now();
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_controller";
  status.message = "attribute measurement";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  
  // Width
  diagnostic_msgs::msg::KeyValue kv_width;
  kv_width.key = "current-corridor-width";
  kv_width.value = std::to_string(width);
  status.values.push_back(kv_width);
  
  // ✅ ADD: Lighting level (1.0 = lit, 0.0 = dark)
  diagnostic_msgs::msg::KeyValue kv_light;
  kv_light.key = "current-lighting-level";
  kv_light.value = is_dark ? "0" : "1"; 
  status.values.push_back(kv_light);
  
  diag_msg->status.push_back(status);
  diagnostics_pub_->publish(std::move(diag_msg));
  
  RCLCPP_INFO(get_logger(), 
    "✅ Published corridor safety: width=%.2fm, lighting=%s (%s→%s)", 
    width, is_dark ? "DARK" : "LIT", from_wp.c_str(), to_wp.c_str());
}


std::string NavigationController::getClosestWaypointInCorridor(
  const std::string& wp_a, 
  const std::string& wp_b)
{
  if (!latest_amcl_pose_received_) {
    RCLCPP_WARN(get_logger(), 
      "⚠️ No AMCL pose available, defaulting to %s", wp_a.c_str());
    return wp_a;
  }

  geometry_msgs::msg::Pose robot_pose = amcl_pose_.pose.pose;
  
  std::map<std::string, std::pair<double, double>> waypoints = {
    {"wp_0",  {0.0, 0.0}},
    {"wp_1",  {-4.7, -2.5}},
    {"wp_2",  {-7.7, 0.0}},
    {"wp_3",  {-8.0, 5.5}},
    {"wp_4",  {-1.53841, 4.0}},
    {"wp_5",  {-1.0, 0.0}},
    {"wp_6",  {-8.0, -1.5}},
    {"wp_7",  {-8.0, 3.5}},
    {"wp_8",  {0.0, 6.0}},
    {"wp_9",  {-6.5, -1.5}},
    {"wp_10", {-6.0, 4.5}}
  };
  
  auto it_a = waypoints.find(wp_a);
  auto it_b = waypoints.find(wp_b);
  
  if (it_a == waypoints.end() || it_b == waypoints.end()) {
    RCLCPP_ERROR(get_logger(), 
      "❌ Waypoint coordinates not found for %s or %s!", 
      wp_a.c_str(), wp_b.c_str());
    return wp_a;
  }
  
  double dist_a = std::hypot(
    robot_pose.position.x - it_a->second.first,
    robot_pose.position.y - it_a->second.second
  );
  
  double dist_b = std::hypot(
    robot_pose.position.x - it_b->second.first,
    robot_pose.position.y - it_b->second.second
  );
  
  // ✅ SAFETY CHECK: Detect wacky AMCL
  double corridor_length = std::hypot(
    it_b->second.first - it_a->second.first,
    it_b->second.second - it_a->second.second
  );
  
  double max_reasonable_distance = corridor_length * 2.0;
  
  if (dist_a > max_reasonable_distance && dist_b > max_reasonable_distance) {
    RCLCPP_ERROR(get_logger(), 
      "⚠️ AMCL WACKY! Robot %.2fm from %s and %.2fm from %s", 
      dist_a, wp_a.c_str(), dist_b, wp_b.c_str());
    RCLCPP_WARN(get_logger(), 
      "   Corridor length: %.2fm, defaulting to %s", 
      corridor_length, wp_a.c_str());
    return wp_a;
  }
  
  RCLCPP_INFO(get_logger(), "🧭 AMCL: x=%.2f, y=%.2f", 
    robot_pose.position.x, robot_pose.position.y);
  RCLCPP_INFO(get_logger(), "   %s: %.2f m | %s: %.2f m", 
    wp_a.c_str(), dist_a, wp_b.c_str(), dist_b);
  
  std::string closest = (dist_a < dist_b) ? wp_a : wp_b;
  RCLCPP_INFO(get_logger(), "📍 Closest: %s", closest.c_str());
  
  return closest;
}

void NavigationController::fetch_component_states()
{
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = 
    "match "
    "$c isa Component, has component-name $name, has component-status $status; "
    "fetch $name; $status;";  // ← Changed from is-active to component-status

  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(500ms) != std::future_status::ready) return;
  
  auto resp = fut.get();
  if (!resp->success) return;

  for (const auto &row : resp->results) {
    std::string name;
    std::string status = "feasible";  // ← Changed from bool to string
    
    for (const auto &attr : row.attributes) {
      if (attr.name == "name") name = attr.value.string_value;
      if (attr.name == "status") status = attr.value.string_value;  // ← STRING!
    }
    
    // ✅ Check if status is 'unfeasible' or 'failure'
    bool active = (status != "unfeasible" && status != "failure");
    
    if (name == "perception_node") {
      perception_active_ = active;
    }
    else if (name == "arm_controller") {
      arm_active_ = active;
    }
  }
  
  // RCLCPP_DEBUG(get_logger(), 
  //   "Component states: Perception=%s, Arm=%s",
  //   perception_active_ ? "ON" : "OFF",
  //   arm_active_ ? "HIGH" : "LOW");
}
// Helper: parse action string
// -----------------------------------------------------------------------------
std::tuple<std::string, std::string, std::string>
NavigationController::parse_action(const std::string &action)
{
  // Expected format: "(move_lit move_lit wp_1 wp_2 high_speed_config)"
  //                    ^action  ^param   ^from ^to  ^config
  std::stringstream ss(action);
  std::string action_name, action_param, from, to, cfg;
  
  ss >> action_name >> action_param >> from >> to >> cfg;

  // Remove parentheses
  auto clean = [](std::string &s) {
    s.erase(std::remove(s.begin(), s.end(), '('), s.end());
    s.erase(std::remove(s.begin(), s.end(), ')'), s.end());
  };

  clean(action_name);
  clean(action_param);  // ✅ NEW: Clean the action parameter too
  clean(from);
  clean(to);
  clean(cfg);

  if (from.empty() || to.empty() || cfg.empty()) {
    RCLCPP_ERROR(get_logger(), 
      "Failed to parse action: '%s' -> action=%s, param=%s, from='%s', to='%s', cfg='%s'",
      action.c_str(), action_name.c_str(), action_param.c_str(), 
      from.c_str(), to.c_str(), cfg.c_str());
  }

  return {from, to, cfg};
}

void NavigationController::resumeNavigation()
{
  RCLCPP_INFO(get_logger(), "🔋 Recharge complete, resuming navigation!");
  
  std::string current_wp = getCurrentWaypointFromFeedback();
  
  problem_expert_->clearKnowledge();
  build_problem_from_kb();
  problem_expert_->addPredicate(plansys2::Predicate("(at " + current_wp + ")"));
  fetch_goal();  // Restore original goal (wp_10)
  
  auto plan = planner_client_->getPlan(domain_expert_->getDomain(), problem_expert_->getProblem());
  
  if (plan.has_value()) {
    current_goal_ = "navigation";
    recharge_completed_ = false;
    executor_client_->start_plan_execution(plan.value());
  }
}

void NavigationController::publishBatteryMeasurement(double battery_level, const std::string& reason)
{
  // ═══════════════════════════════════════════════════════════
  // PUBLISH TO /diagnostics (ROSA reads this!)
  // ═══════════════════════════════════════════════════════════
  auto diag_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  diag_msg->header.stamp = this->now();
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_controller";
  status.message = "attribute measurement";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  
  // Battery level
  diagnostic_msgs::msg::KeyValue kv_battery;
  kv_battery.key = "battery-level";
  kv_battery.value = std::to_string(battery_level);
  status.values.push_back(kv_battery);

  diag_msg->status.push_back(status);
  diagnostics_pub_->publish(std::move(diag_msg));  // ← ROSA reads this!

  // // ✅ ADD THIS - TRIGGER ROSA TO EVALUATE CONSTRAINTS!
  // auto rosa_msg = std::make_unique<std_msgs::msg::String>();
  // rosa_msg->data = "insert_monitoring_data";
  // rosa_event_pub_->publish(std::move(rosa_msg));
  // rclcpp::sleep_for(std::chrono::milliseconds(300));

  // ═══════════════════════════════════════════════════════════
  // PUBLISH TO /navigation/monitoring (benchmark_monitor reads this)
  // ═══════════════════════════════════════════════════════════
  auto monitor_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  monitor_msg->header.stamp = this->now();
  
  diagnostic_msgs::msg::DiagnosticStatus monitor_status;
  monitor_status.name = "navigation_monitoring";
  monitor_status.message = "adaptation state";
  
  // Config
  diagnostic_msgs::msg::KeyValue kv_config;
  kv_config.key = "current-configuration";
  kv_config.value = current_config_;
  monitor_status.values.push_back(kv_config);
  
  // Reason
  diagnostic_msgs::msg::KeyValue kv_reason;
  kv_reason.key = "adaptation-reason";
  kv_reason.value = reason;
  monitor_status.values.push_back(kv_reason);
  
  // Component states (for benchmark tracking)
  diagnostic_msgs::msg::KeyValue kv_perception;
  kv_perception.key = "perception-active";
  kv_perception.value = perception_active_ ? "true" : "false";
  monitor_status.values.push_back(kv_perception);
  
  // ✅ ADD: Arm state
  diagnostic_msgs::msg::KeyValue kv_arm;
  kv_arm.key = "arm-power-level";
  kv_arm.value = arm_active_ ? "high" : "low";
  monitor_status.values.push_back(kv_arm);
  
  monitor_msg->status.push_back(monitor_status);
  monitoring_pub_->publish(std::move(monitor_msg));
  
  // RCLCPP_DEBUG(get_logger(), 
  //   "📊 Published: battery=%.1f%%, perception=%s, arm=%s", 
  //   battery_level,
  //   perception_active_ ? "ON" : "OFF",
  //   arm_active_ ? "HIGH" : "LOW");
}

void NavigationController::triggerRecharge()
{
  RCLCPP_WARN(get_logger(), "⚠️ Battery too low, initiating recharge!");
  
  executor_client_->cancel_plan_execution();
  rclcpp::sleep_for(500ms);

  std::string current_wp = getCurrentWaypointFromFeedback();
  std::string charger_wp = findNearestCharger(current_wp);

  problem_expert_->clearKnowledge();
  build_problem_from_kb();
  problem_expert_->addPredicate(plansys2::Predicate("(at " + current_wp + ")"));
  problem_expert_->setGoal(plansys2::Goal("(and (battery_recharged " + charger_wp + "))"));
  
  auto plan = planner_client_->getPlan(domain_expert_->getDomain(), problem_expert_->getProblem());
  
  if (plan.has_value()) {
    current_goal_ = "recharge";
    executor_client_->start_plan_execution(plan.value());
  }
}

void NavigationController::step()
{
  if (battery_level_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), 
      "💀 BATTERY DEPLETED (%.1f%%)! Mission failed!", battery_level_);
    
    // ✅ CANCEL EXECUTOR IMMEDIATELY!
    executor_client_->cancel_plan_execution();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ✅ Clear problem expert (remove stale predicates)
    problem_expert_->clearKnowledge();
    
    // ✅ Reset flags that might interfere
    current_goal_ = "navigation";
    pending_recharge_ = false;
    recharge_completed_ = false;
    target_charger_ = "";
    current_plan_actions_.clear();
    cost_map_.clear();
    
    // ✅ Mark current run as FAILED
    if (benchmark_mode_) {
      current_run_data_.success = false;
      current_run_data_.failure_reason = "battery_depleted";
      current_run_data_.battery_end = 0.0;
    }
    
    // ✅ Let finish_controlling() handle the restart
    finish_controlling();
    return;
  }
  if (!executor_client_->execute_and_check_plan()) {
    auto result = executor_client_->getResult();
    if (result.has_value() && current_goal_ == "navigation") {
      RCLCPP_INFO(get_logger(), "✅ Navigation complete! Stopping controller.");
      finish_controlling();
      return;  // ← Exit immediately, skip all other processing!
    }
  }
  // ═══════════════════════════════════════════════
  //  First iteration: start normal navigation plan
  // ═══════════════════════════════════════════════
  if (first_iteration_) {
    execute_plan();
    first_iteration_ = false;
    return;
  }
  if (!executor_client_->execute_and_check_plan() && 
      executor_client_->getResult()) {
    auto result = executor_client_->getResult();
    if (result.has_value()) {
      RCLCPP_INFO(get_logger(), "✅ Plan finished");
      
      // ✅ If we just finished recharging, wait for resume logic
      if (current_goal_ == "recharge") {
        RCLCPP_INFO(get_logger(), 
          "🔋 Recharge plan complete, will resume navigation...");
        return;  // Next iteration handles resume
      }
      
      // ✅ Navigation completed
      finish_controlling();
    }
    return;
  }
  // ═══════════════════════════════════════════════
  //  1. Handle recharge COMPLETION and resume navigation
  // ═══════════════════════════════════════════════
  if (current_goal_ == "recharge") {
    // Check if we're actually performing the recharge action (not just moving to charger)
    auto feedback = executor_client_->getFeedBack();
    bool is_recharging = false;
    
    for (const auto &ae : feedback.action_execution_status) {
      if (ae.action == "recharge" && 
          ae.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
        is_recharging = true;
        break;
      }
    }
    
    if (!is_recharging && !recharge_completed_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "⏳ Waiting for recharge to start...");
      return;  // Still moving to charger
    }
    
    // Only proceed with resume if recharge completed
    if (!recharge_completed_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "⏳ Waiting for recharge to finish...");
      return;
    }

    // ✅ Get charger location from stored target_charger_
    std::string charger_wp = target_charger_;
  
    if (charger_wp.empty()) {
      RCLCPP_ERROR(get_logger(), 
        "❌ target_charger_ was not set! Falling back...");
      
      // Try to get from feedback as fallback
      for (const auto &ae : feedback.action_execution_status) {
        if (ae.action == "move_to_recharge" &&
            ae.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED &&
            ae.arguments.size() >= 3) {
          charger_wp = ae.arguments[2];
          break;
        }
      }
      
      if (charger_wp.empty()) {
        charger_wp = "wp_0";  // Emergency fallback
      }
    }
    
    std::string last_known_wp_ = charger_wp;
    
    RCLCPP_INFO(get_logger(),
      "🔁 Recharge finished at %s — resuming navigation!", last_known_wp_.c_str());

    // ✅ Clean up executor state
    executor_client_->cancel_plan_execution();
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    auto exec_status = executor_client_->getOrderedSubGoals();
    int retries = 0;
    while (!exec_status.empty() && retries < 5) {
      RCLCPP_WARN(get_logger(), 
        "⏳ Executor has %zu goals, clearing... (retry %d/5)", 
        exec_status.size(), retries + 1);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      exec_status = executor_client_->getOrderedSubGoals();
      retries++;
    }

    // ✅ Give actions time to stabilize
    RCLCPP_INFO(get_logger(), "⏸️  Waiting for action nodes to stabilize...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // ✅ Publish updated battery to ROSA
    RCLCPP_INFO(get_logger(),
      "🔋 Publishing recharged battery level: %.2f%%", battery_level_);
    publishBatteryMeasurement(battery_level_, "post_recharge");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ✅ Rebuild problem from charger location
    problem_expert_->clearKnowledge();
    fetch_actions();
    fetch_waypoints();
    fetch_corridors();
    fetch_configurations();
    fetch_charging_stations();
    fetch_lighting_conditions();
    fetch_energy_costs();
    fetch_battery_and_feasibility();

    // ✅ Robot is now at charger
    problem_expert_->addPredicate(
      plansys2::Predicate("(at " + last_known_wp_ + ")"));

    // ✅ Restore original navigation goal
    fetch_goal();
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // ✅ Generate new plan from charger to goal
    auto domain  = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan    = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), "❌ Failed to replan after recharge");
      return;
    }

    RCLCPP_INFO(get_logger(), "✅ Resumed plan: %zu actions from %s", 
      plan->items.size(), last_known_wp_.c_str());
    
    
    RCLCPP_INFO(get_logger(), "📊 Plan actions populated: %zu", current_plan_actions_.size());
    
    // ✅ Final check before starting
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    
    auto final_check = executor_client_->getOrderedSubGoals();
    if (!final_check.empty()) {
      RCLCPP_ERROR(get_logger(), 
        "⚠️ Executor STILL has %zu goals right before start!", final_check.size());
      executor_client_->cancel_plan_execution();
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    
    // ✅ Start execution
    RCLCPP_INFO(get_logger(), "🚀 Starting plan execution...");
    executor_client_->start_plan_execution(plan.value());
    
    rclcpp::sleep_for(std::chrono::milliseconds(1000));  // Let actions activate

    // ✅ Reset state
    current_goal_ = "navigation";
    recharge_completed_ = false;
    last_action.clear();
    target_charger_ = "";  // Clear for next time

    RCLCPP_INFO(get_logger(), "▶️ Resuming navigation from %s", last_known_wp_.c_str());
  
    return;
  }

  // ═══════════════════════════════════════════════
  //  2. Check if we need to initiate recharge (REACTIVE)
  // ═══════════════════════════════════════════════
  if (pending_recharge_) {
    RCLCPP_WARN(get_logger(), "🔋 Battery critically low - INITIATING RECHARGE!");
    pending_recharge_ = false;
    recharge_completed_ = false;
    
    // Cancel current plan
    executor_client_->cancel_plan_execution();
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ✅ Get current position
    auto feedback = executor_client_->getFeedBack();
    std::string from_wp, to_wp;
    
    // Try to get from executing action
    for (const auto &ae : feedback.action_execution_status) {
      if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
        if ((ae.action == "move_lit" || ae.action == "move_dark") && 
            ae.arguments.size() >= 3) {
          from_wp = ae.arguments[1];
          to_wp = ae.arguments[2];
          RCLCPP_INFO(get_logger(), 
            "🔋 Robot in corridor: %s → %s", from_wp.c_str(), to_wp.c_str());
          break;
        }
      }
    }
    
    // Use AMCL to pick closest endpoint
    if (!from_wp.empty() && !to_wp.empty()) {
      last_known_wp_ = getClosestWaypointInCorridor(from_wp, to_wp);
    } else {
      RCLCPP_WARN(get_logger(), "No executing action, using fallback");
      last_known_wp_ = getCurrentWaypointFromFeedback();
    }
    
    RCLCPP_INFO(get_logger(), 
      "🔋 Robot at %s, planning recharge route", last_known_wp_.c_str());
    
    // ✅ Rebuild problem from current position
    problem_expert_->clearKnowledge();
    fetch_actions();   
    fetch_waypoints();
    fetch_corridors();
    fetch_configurations();
    
    // ✅ Switch to safest config for recharge route
    auto feasible_cfgs = getFeasibleConfigsFromKB();
    if (!feasible_cfgs.empty()) {
      current_config_ = feasible_cfgs[0];
      RCLCPP_WARN(get_logger(), 
        "⚡ Switched to emergency config: %s", current_config_.c_str());
      
      // Publish config change
      auto monitor_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
      monitor_msg->header.stamp = this->now();
      
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "navigation_monitoring";
      status.message = "adaptation state";
      
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "current-configuration";
      kv.value = current_config_;
      status.values.push_back(kv);
      
      diagnostic_msgs::msg::KeyValue kv_reason;
      kv_reason.key = "adaptation-reason";
      kv_reason.value = "emergency_recharge";
      status.values.push_back(kv_reason);
      
      monitor_msg->status.push_back(status);
      monitoring_pub_->publish(std::move(monitor_msg));
      
      rclcpp::sleep_for(std::chrono::milliseconds(250));
    }

    fetch_charging_stations();
    fetch_lighting_conditions();
    fetch_energy_costs();
    fetch_battery_and_feasibility();

    // ✅ Set robot at current location
    problem_expert_->addPredicate(
      plansys2::Predicate("(at " + last_known_wp_ + ")"));

    // ✅ Find nearest charger and store it
    target_charger_ = findNearestCharger(last_known_wp_);

    RCLCPP_INFO(get_logger(), 
      "📍 Routing to charger: %s", target_charger_.c_str());

    // ✅ Set goal to reach that charger
    problem_expert_->setGoal(
      plansys2::Goal("(and (battery_recharged " + target_charger_ + "))"));
    
    current_goal_ = "recharge";

    // Give PlanSys2 time to update
    rclcpp::sleep_for(std::chrono::milliseconds(400));

    // ✅ Compute recharge plan
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), 
        "❌ No recharge plan from %s", last_known_wp_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), 
      "✅ Recharge route planned from %s", last_known_wp_.c_str());
    
    // computeTotalCost(plan.value());
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // ✅ Execute recharge plan
    executor_client_->start_plan_execution(plan.value());
    return;
  }

  // ═══════════════════════════════════════════════
  //  3. Query ROSA for feasible actions (REACTIVE)
  // ═══════════════════════════════════════════════
  auto feasible_actions = getFeasibleActionsFromRosa();

  bool move_feasible =
    isActionFeasible(feasible_actions, "move_lit") ||
    isActionFeasible(feasible_actions, "move_dark");
  bool recharge_feasible = isActionFeasible(feasible_actions, "recharge");

  // ✅ ADD THIS LINE:
  bool move_to_recharge_feasible = isActionFeasible(feasible_actions, "move_to_recharge");

  // ═══════════════════════════════════════════════
  //  4. REACTIVE TRIGGER: move_to_recharge available
  // ═══════════════════════════════════════════════
  if (move_to_recharge_feasible &&  // ← CHANGE THIS LINE
      current_goal_ != "recharge" && 
      !pending_recharge_) {
    
    RCLCPP_WARN(get_logger(), "⚠️ REACTIVE: Battery ≤ 15%% - move_to_recharge available!");
    
    std::string current_position = getCurrentWaypointFromFeedback();
    
    if (current_position.empty()) {
      RCLCPP_ERROR(get_logger(), "❌ Cannot determine current position!");
      return;
    }
    
    target_charger_ = findNearestCharger(current_position);
    
    RCLCPP_ERROR(get_logger(), 
      "🔴 SECTION 4: SET target_charger_ = '%s' (from position: %s)", 
      target_charger_.c_str(), current_position.c_str());
    
    if (target_charger_.empty()) {
      RCLCPP_ERROR(get_logger(), "❌ Failed to find charger!");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "🔄 Will recharge at %s",
      target_charger_.c_str());
    
    pending_recharge_ = true;
    return;
  }

  // ═══════════════════════════════════════════════
  //  5. Check if plan completed
  // ═══════════════════════════════════════════════
//   if (!executor_client_->execute_and_check_plan() && 
//       executor_client_->getResult()) {
//     auto result = executor_client_->getResult();
//     if (result.has_value()) {
//       RCLCPP_INFO(get_logger(), "✅ Plan finished");
      
//       // ✅ If we just finished recharging, wait for resume logic
//       if (current_goal_ == "recharge") {
//         RCLCPP_INFO(get_logger(), 
//           "🔋 Recharge plan complete, will resume navigation...");
//         return;  // Next iteration handles resume
//       }
      
//       // ✅ Navigation completed
//       finish_controlling();
//     }
//     return;
//   }

  // ═══════════════════════════════════════════════
  //  6. Publish REAL battery level to ROSA
  // ═══════════════════════════════════════════════
  fetch_component_states();   
  publishBatteryMeasurement(battery_level_, "battery_update");
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  
  // ═══════════════════════════════════════════════
  //  7. UNIFIED CONFIG ADAPTATION (with cooldown)
  // ═══════════════════════════════════════════════
  
  // ✅ Check cooldown FIRST (applies to both types)
  auto now = std::chrono::steady_clock::now();
  auto seconds_since_change = std::chrono::duration_cast<std::chrono::seconds>(
    now - last_config_change_time_).count();
  
  bool cooldown_active = (seconds_since_change < CONFIG_CHANGE_COOLDOWN_SEC);
  
  if (cooldown_active) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "⏸️ Config change blocked - cooldown active (%ld/%d sec)",
      seconds_since_change, CONFIG_CHANGE_COOLDOWN_SEC);
    goto skip_adaptation;  // Skip both planned AND battery adaptations
  }
  
  // ─────────────────────────────────────────────
  // 7A. Check if CURRENT ACTION requires different config
  // ─────────────────────────────────────────────
  if (!current_plan_actions_.empty()) {
    auto feedback = executor_client_->getFeedBack();
    
    for (size_t i = 0; i < feedback.action_execution_status.size() && 
                       i < current_plan_actions_.size(); i++) {
      const auto &ae = feedback.action_execution_status[i];
      const auto &pa = current_plan_actions_[i];
      
      // Only check EXECUTING action
      if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
        
        if (!pa.config.empty() && pa.config != current_config_) {
          
          // ✅ Check if planned config is still feasible (battery constraint)
          auto feasible_cfgs = getFeasibleConfigsFromKB();
          bool is_safe = std::find(feasible_cfgs.begin(), feasible_cfgs.end(), pa.config) 
                         != feasible_cfgs.end();
          
          if (!is_safe) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "⚠️ Action wants '%s' but battery blocks it! Staying in '%s'",
              pa.config.c_str(), current_config_.c_str());
            break;  // ✅ Don't adapt, keep current config
          }
          
          // ✅ SAFE TO ADAPT - Apply planned config
          RCLCPP_WARN(get_logger(), 
            "🔄 PLANNED: '%s' → '%s' (corridor constraint)",
            current_config_.c_str(), pa.config.c_str());
          
          current_config_ = pa.config;
          last_config_change_time_ = now;  // ✅ Reset cooldown timer
          
          // Publish monitoring message
          auto monitor_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
          monitor_msg->header.stamp = this->now();
          
          diagnostic_msgs::msg::DiagnosticStatus status;
          status.name = "navigation_monitoring";
          status.message = "adaptation state";
          
          diagnostic_msgs::msg::KeyValue kv_config;
          kv_config.key = "current-configuration";
          kv_config.value = current_config_;
          status.values.push_back(kv_config);
          
          diagnostic_msgs::msg::KeyValue kv_reason;
          kv_reason.key = "adaptation-reason";
          kv_reason.value = "planned_safety_constraint";
          status.values.push_back(kv_reason);
          
          monitor_msg->status.push_back(status);
          monitoring_pub_->publish(std::move(monitor_msg));
          
          // Trigger ROSA
          auto rosa_msg = std::make_unique<std_msgs::msg::String>();
          rosa_msg->data = "insert_monitoring_data";
          rosa_event_pub_->publish(std::move(rosa_msg));
          
          goto skip_adaptation;  // ✅ Skip battery adaptation this iteration
        }
        break;  // Only check currently executing action
      }
    }
  }
  
  // ─────────────────────────────────────────────
  // 7B. Check if BATTERY requires different config
  // ─────────────────────────────────────────────
  {
    auto feasible_cfgs = getFeasibleConfigsFromKB();
    
    if (feasible_cfgs.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "⚠️ No feasible configs available!");
      goto skip_adaptation;
    }
    
    // ✅ FIX: Only adapt if current config became UNFEASIBLE
    bool current_still_feasible = 
      std::find(feasible_cfgs.begin(), feasible_cfgs.end(), current_config_) 
      != feasible_cfgs.end();
    
    if (!current_still_feasible) {
      // Current config is no longer safe - must switch!
      std::string best_config = feasible_cfgs[0];
      
      RCLCPP_WARN(get_logger(),
        "🔄 REACTIVE: '%s' → '%s' (battery constraint triggered)",
        current_config_.c_str(), best_config.c_str());
      
      current_config_ = best_config;
      last_config_change_time_ = now;  // ✅ Reset cooldown timer
      
      // Publish monitoring message
      auto monitor_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
      monitor_msg->header.stamp = this->now();
      
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "navigation_monitoring";
      status.message = "adaptation state";
      
      diagnostic_msgs::msg::KeyValue kv_config;
      kv_config.key = "current-configuration";
      kv_config.value = current_config_;
      status.values.push_back(kv_config);
      
      diagnostic_msgs::msg::KeyValue kv_reason;
      kv_reason.key = "adaptation-reason";
      kv_reason.value = "battery_constraint";
      status.values.push_back(kv_reason);
      
      monitor_msg->status.push_back(status);
      monitoring_pub_->publish(std::move(monitor_msg));
      
      // // Trigger ROSA
      // auto rosa_msg = std::make_unique<std_msgs::msg::String>();
      // rosa_msg->data = "insert_monitoring_data";
      // rosa_event_pub_->publish(std::move(rosa_msg));
    }
  }

skip_adaptation:


  // ═══════════════════════════════════════════════
  //  8. Track last action (for debugging)
  // ═══════════════════════════════════════════════
  auto feedback = executor_client_->getFeedBack();
  bool found_executing = false;
  for (const auto &ae : feedback.action_execution_status) {
    if (ae.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
      last_action = ae.action;
      found_executing = true;
      break;
    }
  }
  if (!found_executing) {
    last_action.clear();
  }

} // END OF step() function




void NavigationController::initializeBenchmarkRun()
{
  current_run_data_ = BenchmarkRunData();
  current_run_data_.run_number = current_run_;
  current_run_data_.start_time = std::to_string(this->now().seconds());
  current_run_data_.success = true;
  current_run_data_.battery_start = 100.0;
  current_run_data_.move_actions = 0;
  current_run_data_.recharge_count = 0;
  current_run_data_.total_distance = 0.0;
  current_run_data_.num_config_adaptations = 0;
  current_run_data_.num_component_adaptations = 0;
  current_run_data_.emergency_events = 0;
  current_run_data_.min_battery_reached = 100.0;
}

void NavigationController::publishBenchmarkData()
{
  // Finalize run data
  current_run_data_.end_time = std::to_string(this->now().seconds());
  double start = std::stod(current_run_data_.start_time);
  double end = std::stod(current_run_data_.end_time);
  current_run_data_.duration = end - start;
  current_run_data_.battery_end = battery_level_;
  current_run_data_.battery_used = current_run_data_.battery_start - battery_level_;
  
  // Query map characteristics from TypeDB
  auto req = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  req->query_type = "fetch";
  req->query = "match $wp isa waypoint; get $wp; count;";
  
  auto fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(500ms) == std::future_status::ready) {
    auto resp = fut.get();
    current_run_data_.total_waypoints = resp->results.size();
  }
  
  // Count chargers
  req->query = "match $wp isa waypoint, has has-charging-station true; get $wp; count;";
  fut = typedb_query_cli_->async_send_request(req);
  if (fut.wait_for(500ms) == std::future_status::ready) {
    auto resp = fut.get();
    current_run_data_.num_chargers = resp->results.size();
  }
  
  // Save to vector
  all_run_data_.push_back(current_run_data_);
  
  // Publish to ROS topic (for Python monitor)
  auto msg = std_msgs::msg::String();
  msg.data = "Run " + std::to_string(current_run_) + " complete";
  benchmark_data_pub_->publish(msg);
  
  RCLCPP_INFO(get_logger(), 
    "📊 Run %d: Duration=%.1fs, Battery=%.1f%%, Moves=%d, Recharges=%d",
    current_run_data_.run_number,
    current_run_data_.duration,
    current_run_data_.battery_used,
    current_run_data_.move_actions,
    current_run_data_.recharge_count);
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
