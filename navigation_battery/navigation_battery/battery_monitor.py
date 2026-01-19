#!/usr/bin/env python3


import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Int32
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, LookupException

try:
    from plansys2_msgs.msg import ActionExecution, PlanExecutionStatus
except ImportError:
    ActionExecution = None
    PlanExecutionStatus = None


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # ═════════════════ PARAMETERS ═════════════════
        self.declare_parameter('benchmark_mode', False)
        self.declare_parameter('benchmark_seed', 42)

        self.declare_parameter('drain_high_speed', 2.0)
        self.declare_parameter('drain_low_speed', 1.2)
        self.declare_parameter('drain_degraded', 0.8)
        self.declare_parameter('drain_idle', 0.1)

        self.declare_parameter('initial_level', 100.0)
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('recharge_rate', 1.0)

        # Load parameters
        self.benchmark = self.get_parameter('benchmark_mode').value
        self.seed = self.get_parameter('benchmark_seed').value

        self.rate_high = self.get_parameter('drain_high_speed').value
        self.rate_low = self.get_parameter('drain_low_speed').value
        self.rate_degraded = self.get_parameter('drain_degraded').value
        self.rate_idle = self.get_parameter('drain_idle').value

        self.battery = self.get_parameter('initial_level').value
        self.publish_period = self.get_parameter('publish_period').value
        self.recharge_rate = self.get_parameter('recharge_rate').value

        # ═════════════════ INTERNAL STATE ═════════════════
        self.current_config = "high_speed_config"
        self.perception = True
        self.arm_level = "high"
        self.is_moving = False

        self.recharging = False
        self.recharge_done = False

        # Plan lifecycle: freeze drain when plan succeeded
        self.plan_finished = False  # set by PlanSys2 executor status

        # Startup grace period
        self.grace_active = True
        self.grace_duration = 14.0
        self.grace_start = self.get_clock().now()

        # Post-charge grace
        self.post_charge_grace = False
        self.post_charge_start = None

        # Timing
        self.last_time = self.get_clock().now()
        self.prev_pose = None 

        if self.benchmark:
            random.seed(self.seed)

        # ═════════════════ ROS SETUP ═════════════════
        self.pub_batt = self.create_publisher(BatteryState, "/battery_state", 10)
        self.pub_recharge_done = self.create_publisher(Bool, "/battery_monitor/recharge_complete", 10)

        self.reset_srv = self.create_service(Trigger, "/battery_monitor/reset", self.reset_battery)

        self.sub_recharge = self.create_subscription(
            Bool, "/battery_monitor/recharge", self.recharge_cb, 10
        )

        # ⬅️ Monitoring from navigation / ROSA
        self.sub_monitoring = self.create_subscription(
            DiagnosticArray,
            "/navigation/monitoring",
            self.monitoring_cb,
            10
        )

        # Benchmark run completion (your fake runs)
        self.sub_run_complete = self.create_subscription(
            Int32, "/benchmark/run_complete", self.run_complete_cb, 10
        )

        # Movement from PlanSys2 action execution (optional)
        if self.benchmark and ActionExecution is not None:
            self.sub_action = self.create_subscription(
                ActionExecution,
                "/action_execution_info",
                self.action_cb,
                10
            )

        # Plan-level status from PlanSys2 executor (NEW)
        if PlanExecutionStatus is not None:
            # Adjust topic name if your executor uses a different one
            self.sub_plan_status = self.create_subscription(
                PlanExecutionStatus,
                "/plansys2_executor/status",
                self.plan_status_cb,
                10
            )
            self.get_logger().info("📡 Subscribed to /plansys2_executor/status for plan lifecycle")
        else:
            self.get_logger().warn("⚠️ plansys2_msgs.PlanExecutionStatus not available; "
                                   "plan-based freezing disabled.")

        if not self.benchmark:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.publish_period, self.update)

        mode = "🔬 BENCHMARK" if self.benchmark else "🎮 SIMULATION"
        self.get_logger().info(f"🔋 Battery Monitor running in {mode} mode")
        self.get_logger().info(f"   Initial battery: {self.battery:.1f}%")

    # ═════════════════ CALLBACKS ═════════════════

    def monitoring_cb(self, msg: DiagnosticArray):
        """Handle config + component states from ROSA."""
        for status in msg.status:
            if status.name != "navigation_monitoring":
                continue

            for kv in status.values:

                # CONFIG
                if kv.key == "current-configuration":
                    new = kv.value
                    if new != self.current_config:
                        self.get_logger().info(
                            f"⚙️ Config: {self.current_config} → {new}"
                        )
                        self.current_config = new

                # MOVEMENT (optional, often overridden by /action_execution_info)
                elif kv.key == "is_moving":
                    self.is_moving = kv.value.lower() == "true"

                # PERCEPTION
                elif kv.key == "perception-active":
                    self.perception = kv.value.lower() == "true"

                # ARM POWER
                elif kv.key == "arm-power-level":
                    self.arm_level = kv.value

                # ADAPTATION REASON — ignored (no spam)
                elif kv.key == "adaptation-reason":
                    pass

    def action_cb(self, msg: ActionExecution):
        """PlanSys2 movement detection."""
        name = msg.action.lower()

        if "move" in name:
            if msg.status == 1 and not self.is_moving:
                self.is_moving = True
            elif msg.status in [2, 3, 4] and self.is_moving:
                self.is_moving = False

    def plan_status_cb(self, msg: 'PlanExecutionStatus'):
        """
        Track PlanSys2 executor status and freeze draining when plan is finished.

        We treat SUCCEEDED as 'finished'. You can optionally also freeze on FAILURE/CANCELLED.
        """
        status_val = msg.status

        # For debugging / verification (first runs):
        # self.get_logger().info(f"📡 Plan status update: {status_val}")

        # Prefer symbolic constant if available
        succeeded_val = getattr(PlanExecutionStatus, "SUCCEEDED", None)

        if succeeded_val is not None:
            if status_val == succeeded_val:
                if not self.plan_finished:
                    self.plan_finished = True
                    self.get_logger().info("🎯 Plan SUCCEEDED → freezing battery drain until next run reset")
            else:
                # Any non-succeeded state means we are in planning/executing/etc.
                if self.plan_finished:
                    self.plan_finished = False
        else:
            # Fallback: assume '3' is SUCCEEDED (common in PlanSys2)
            if status_val == 3:
                if not self.plan_finished:
                    self.plan_finished = True
                    self.get_logger().info("🎯 Plan SUCCEEDED (status=3) → freezing battery drain")
            else:
                if self.plan_finished:
                    self.plan_finished = False

    def recharge_cb(self, msg: Bool):
        self.recharging = msg.data
        if self.recharging:
            self.get_logger().info("⚡ Recharging started...")
        else:
            self.get_logger().info("🔌 Recharging stopped.")

    def run_complete_cb(self, msg: Int32):
        """
        Called by your benchmark runner at end of a run:
        - reset battery
        - reactivate startup-like grace
        - clear plan_finished (new run will start later)
        """
        if not self.benchmark:
            return

        run = msg.data
        self.get_logger().info(f"🔄 Run {run} finished → Battery reset (100%)")

        self.battery = 100.0
        self.recharging = False
        self.recharge_done = False

        # New run will be scheduled; we clear plan_finished
        self.plan_finished = False

        self.grace_active = True
        self.grace_start = self.get_clock().now()

    def reset_battery(self, req, res):
        self.battery = 100.0
        self.recharging = False
        self.recharge_done = False

        # Manual reset also implies "prepare for a new run"
        self.plan_finished = False

        self.grace_active = True
        self.grace_start = self.get_clock().now()

        res.success = True
        res.message = "Battery reset to 100%"
        return res

    # ═════════════════ BATTERY UPDATE ═════════════════

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            return

        # GRACE AT START (startup or after run reset)
        if self.grace_active:
            if (now - self.grace_start).nanoseconds / 1e9 < self.grace_duration:
                self.publish(0.0)
                return
            else:
                self.grace_active = False
                self.get_logger().info("🛡️ Grace period finished — starting drain.")

        # POST-CHARGE GRACE
        if self.post_charge_grace:
            if (now - self.post_charge_start).nanoseconds / 1e9 < self.grace_duration:
                self.publish(0.0)
                return
            else:
                self.post_charge_grace = False
                self.get_logger().info("⚡ Post-charge grace finished.")

        # PLAN FINISHED: freeze draining until reset/run_complete
        if self.plan_finished and not self.recharging:
            # Plan has succeeded; robot is idle from a planning perspective.
            self.publish(0.0)
            return

        # RECHARGING
        if self.recharging:
            # ✅ CHANGE: Instant recharge in BOTH modes (was gradual in simulation)
            self.battery = 100.0
            self.recharging = False
            self.recharge_done = True

            self.post_charge_grace = True
            self.post_charge_start = now

            done = Bool()
            done.data = True
            self.pub_recharge_done.publish(done)

            self.get_logger().info("⚡ Instant recharge complete!")
            self.publish(0.0)
            return


        # NORMAL DRAIN
        if self.benchmark:
            drain = self.benchmark_drain(dt)
        else:
            drain = self.simulation_drain(dt, self.get_distance())

        self.battery = max(0.0, self.battery - drain)
        self.publish(drain)

    # ═════════════════ DRAIN MODELS ═════════════════

    def benchmark_drain(self, dt):
        # ==============================================================
        # CONFIG-DEPENDENT BASE RATE
        # ==============================================================

        if self.current_config == "high_speed_config":
            base = 2.05          
            spike_small = 0.40   
            spike_big = 0.75     
            p_big = 0.15         
            p_small = 0.55       
        elif self.current_config == "low_speed_config":
            base = 1.05
            spike_small = 0.20
            spike_big = 0.45
            p_big = 0.10
            p_small = 0.50
        else:   # degraded
            base = 0.75
            spike_small = 0.10
            spike_big = 0.25
            p_big = 0.05
            p_small = 0.40

        # ==============================================================
        # SPIKE MODEL
        # ==============================================================

        r = random.random()

        if r < p_big:
            spike = spike_big
        elif r < p_big + p_small:
            spike = spike_small
        else:
            spike = 0.0

        # ==============================================================
        # SMALL RANDOM JITTER
        # ==============================================================

        jitter = random.uniform(-0.10, 0.20)

        # ==============================================================
        # COMPUTE MOVEMENT DRAIN
        # ==============================================================

        drain_per_second = base + spike + jitter
        drain_per_second = min(max(drain_per_second, base), base + spike_big + 0.20)
        
        movement_drain = drain_per_second * dt

        # ==============================================================
        # ✅ ADD COMPONENT DRAIN (like simulation mode)
        # ==============================================================
        
        component_drain = 0.0
        
        # Perception: 0.16% per 2sec when ON
        if self.perception:
            component_drain += 0.08 * dt
        
        # Arm: 0.24% per 2sec when HIGH, 0.08% when LOW
        if self.arm_level == "high":
            component_drain += 0.12 * dt
        elif self.arm_level == "low":
            component_drain += 0.04 * dt
        
        return movement_drain + component_drain

    def simulation_drain(self, dt, dist):
        """✅ CHANGE: Slightly more aggressive drain for faster simulations"""
        speed = dist / dt if dt > 0 else 0.0

        if speed < 0.01:
            return self.rate_idle * dt * 1.5  # ✅ 1.5x idle (was 1x)

        # ✅ Slightly higher base rates
        if self.current_config == "high_speed_config":
            base = 0.75  # ✅ Was 0.5
        elif self.current_config == "low_speed_config":
            base = 0.45  # ✅ Was 0.3
        else:
            base = 0.30  # ✅ Was 0.2

        movement = base * dt * (1 + 0.5 * speed)  # ✅ 0.5 multiplier (was 0.3)

        # ✅ Slightly higher component drain
        comp = 0.0
        if self.perception:
            comp += 0.12 * (dt / 100)  # ✅ Was 0.08
        if self.arm_level == "high":
            comp += 0.18 * (dt / 100)  # ✅ Was 0.12
        elif self.arm_level == "low":
            comp += 0.06 * (dt / 100)  # ✅ Was 0.04

        return movement + comp

    def get_distance(self):
        """Calculate distance traveled since last call using TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            current_pose = (x, y)

            if self.prev_pose is not None:
                dx = current_pose[0] - self.prev_pose[0]
                dy = current_pose[1] - self.prev_pose[1]
                distance = math.sqrt(dx*dx + dy*dy)
            else:
                distance = 0.0

            self.prev_pose = current_pose
            return distance
            
        except LookupException:
            # If TF not ready yet, ignore movement
            return 0.0
        except Exception as e:
            self.get_logger().warn(f'TF lookup error: {e}')
            return 0.0

    # ═════════════════ CLEAN LOGGING ═════════════════

    def publish(self, drain):
        msg = BatteryState()
        msg.voltage = 12.0
        msg.percentage = self.battery / 100.0
        msg.present = True
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.recharging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        self.pub_batt.publish(msg)

        mode = "BENCH" if self.benchmark else "SIM"
        state = "Charging" if self.recharging else "Discharging"
        drain_str = f"(-{drain:.3f}%)" if drain > 0 else ""
        perc_str = f"{self.battery:5.1f}%"

        self.get_logger().info(
            f"[{mode}] 🔋 {state} | {perc_str} {drain_str} | "
            f"{self.current_config} | "
            f"👁️ {'ON' if self.perception else 'OFF'} | "
            f"🦾 {self.arm_level.upper()}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()