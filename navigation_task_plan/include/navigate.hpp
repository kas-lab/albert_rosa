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

  #include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


  #include "rclcpp/rclcpp.hpp"
  #include "rclcpp/parameter_client.hpp"

  #include "std_msgs/msg/bool.hpp"
  #include "std_msgs/msg/string.hpp"                 // ⬅ needed for rosa_event_pub_ / rosa_reconfig_sub_
  #include "sensor_msgs/msg/battery_state.hpp"
  #include "std_srvs/srv/trigger.hpp"

  #include "ros_typedb_msgs/srv/query.hpp"

  #include "plansys2_domain_expert/DomainExpertClient.hpp"
  #include "plansys2_executor/ExecutorClient.hpp"
  #include "plansys2_planner/PlannerClient.hpp"
  #include "plansys2_problem_expert/ProblemExpertClient.hpp"
  #include "plansys2_pddl_parser/Utils.h"
  #include "plansys2_msgs/msg/plan.hpp"              // ⬅ computeTotalCost signature

  #include "rosa_task_plan_plansys/rosa_plansys_controller.hpp"
  #include "rosa_msgs/srv/selected_configurations.hpp"
  #include "rosa_msgs/msg/component_configuration.hpp"
  #include "rosa_msgs/srv/action_query_array.hpp"    // ⬅ getFeasibleActionsFromRosa()

  #include <diagnostic_msgs/msg/diagnostic_array.hpp>
  #include <diagnostic_msgs/msg/diagnostic_status.hpp>
  #include <diagnostic_msgs/msg/key_value.hpp>

  namespace navigation_task_plan
  {

  class NavigationController : public rclcpp::Node
  {
  public:
    explicit NavigationController(const std::string & node_name);
    ~NavigationController() override;

  private:
    // --- KB helpers ---
    std::vector<std::string> getFeasibleConfigsFromKB();

    // Current applied configuration (updated on successful adaptation)
    rclcpp::TimerBase::SharedPtr initial_config_timer_;
    std::string current_config_ = "high_speed_config";
    rclcpp::TimerBase::SharedPtr rosa_reasoning_timer_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    // === Goal management ===
    std::string current_goal_ = "navigation";
    std::string findNearestCharger(const std::string &from_wp);

    // === Helper declarations for step() ===
    void checkBatteryPrediction();

    // Feasible actions (ROSA)
    std::vector<std::string> getFeasibleActionsFromRosa();
    bool isActionFeasible(const std::vector<std::string>& feasible_actions,
                          const std::string& action_name);

    // Parse action strings from feedback
    std::tuple<std::string, std::string, std::string> parse_action(const std::string &action);

    // Keep track of last executing action name and last known waypoint (for resume)
    std::string last_action = "";
    std::string last_known_wp_ = "wp_0";            // ⬅ updated on switches & used after recharge

    // Recharge completion handling
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recharge_done_sub_;
    bool recharge_completed_ = false;
    bool enable_proactive_ = false;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose_;
    bool latest_amcl_pose_received_{false};

    std::string getNearestWaypointFromAMCL();

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
    rclcpp::TimerBase::SharedPtr proactive_timer_;
    rclcpp::TimerBase::SharedPtr future_timer_;

    double battery_level_ = 100.0;   // %
    double predicted_cost_ = 0.0;    // %
    double safety_margin_ = 10.0;    // %

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

    // Build PDDL problem from KB
    void build_problem_from_kb();

    // Fetchers (KB → PDDL problem)
    void fetch_actions();
    void fetch_action_feasibility();
    void addAllActionsFeasible();
    void fetch_waypoints();
    void fetch_corridors();
    void fetch_configurations();
    void fetch_lighting_conditions();
    void fetch_energy_costs();
    void fetch_battery_and_feasibility();
    void fetch_goal();

    // Monitoring & adaptation
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

    // Reactive layer (current plan feasibility)
    void evaluatePlanFeasibility();

    // Proactive layer (prediction + replan)
    void updatePredictedBatteryInKB(double predicted_level);
    void evaluateFutureFeasibility();
    void triggerReplan();

    // Architectural adaptation helper
    void triggerProactiveAdaptation(const std::string & target_cfg);
    void updatePlanCostsIfConfigChanged(); 

    // (Optional) simple example function kept for reference
    double computePredictedCost();

    // ROSA integration pubs/subs
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rosa_event_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rosa_reconfig_sub_;

    bool adaptation_triggered_ = false;
    bool task_adaptation_triggered_ = false;
    bool pending_recharge_ = false; 

    // Task adaptation helpers
    void triggerGoalModification();
    std::string findNearestReachableGoal(const std::string &current_wp);
    double calculatePathCost(const std::string &from_wp, const std::string &to_wp);

    // Config helpers
    std::string getCurrentConfigFromKB();
    void updateGoalInKB(const std::string &new_goal);
    void setSystemModeInKB(double mode);
    std::vector<std::string> getAvailableConfigsFromKB();
    void selectConfigurationInKB(const std::string &config_name);

    // ──────────────── NEW: resume + cost calc support ────────────────
    // Read current wp from executor feedback (used when switching/restarting plans)
    std::string getCurrentWaypointFromFeedback();

    // Update KB fact (at <wp>) after recharge / switches
    void updateCurrentWaypointInKB(const std::string &wp);

    // Compute and log total predicted cost for a (re)generated plan
    void computeTotalCost(const plansys2_msgs::msg::Plan &plan);

    // Map for corridor energy costs and current plan action cache
    std::map<std::tuple<std::string, std::string, std::string>, double> cost_map_;
  
    // ✅ ADD THESE TWO LINES:
    std::map<std::pair<std::string, std::string>, double> distance_map_;
    void fetch_corridor_distances();
    void fetch_charging_stations();
    
    
  };

  }  // namespace navigation_task_plan

  #endif  // NAVIGATION_GRAPH__NAVIGATE_CONTROLLER_HPP_
