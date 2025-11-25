#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import Buffer, TransformListener, LookupException


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_proactive_tf')

        # ─────────────── Parameters ───────────────
        self.declare_parameter('base_idle_drain', 0.05)  # ✅ NEW: 0.05% per second when idle
        self.declare_parameter('movement_drain_high', 0.5)  # ✅ 0.5% per second at high speed
        self.declare_parameter('movement_drain_low', 0.3)   # ✅ 0.3% per second at low speed
        self.declare_parameter('movement_drain_degraded', 0.2)  # ✅ 0.2% per second degraded
        
        self.declare_parameter('initial_level', 1.0)
        self.declare_parameter('speed_multiplier', 0.3)  # ✅ Extra drain per m/s of speed
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('recharge_rate', 0.01)

        # ─────────────── Internal State ───────────────
        self.battery_level = float(self.get_parameter('initial_level').value)
        self.base_idle_drain = float(self.get_parameter('base_idle_drain').value)
        self.movement_drain_high = float(self.get_parameter('movement_drain_high').value)
        self.movement_drain_low = float(self.get_parameter('movement_drain_low').value)
        self.movement_drain_degraded = float(self.get_parameter('movement_drain_degraded').value)
        self.speed_multiplier = float(self.get_parameter('speed_multiplier').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.recharge_rate = float(self.get_parameter('recharge_rate').value)
        
        self.current_config = 'high_speed_config'
        self.perception_active = True
        self.arm_power_level = 'high'
        
        self.recharging = False
        self.recharge_complete = False
        self.prev_pose = None
        self.last_time = self.get_clock().now()

        # ─────────────── ROS Interfaces ───────────────
        self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.recharge_complete_pub = self.create_publisher(Bool, '/battery_monitor/recharge_complete', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.recharge_sub = self.create_subscription(
            Bool, '/battery_monitor/recharge', self.recharge_callback, 10
        )
        
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10
        )

        self.timer = self.create_timer(self.publish_period, self.update_battery)
        
        self.get_logger().info(f'🔋 BatteryMonitor running | initial={self.battery_level*100:.1f}%')
        self.get_logger().info(f'📊 Realistic drain model:')
        self.get_logger().info(f'   Idle (stationary): {self.base_idle_drain}%/s')
        self.get_logger().info(f'   Movement high: {self.movement_drain_high}%/s')
        self.get_logger().info(f'   Movement low: {self.movement_drain_low}%/s')
        self.get_logger().info(f'   Movement degraded: {self.movement_drain_degraded}%/s')
        self.get_logger().info(f'   Speed factor: +{self.speed_multiplier}%/s per m/s')
        self.get_logger().info(f'   Perception: +0.08%/100s when ON')
        self.get_logger().info(f'   Arm: HIGH=+0.12%/100s, LOW=+0.04%/100s')

    # ───────────────────────────────────────────────
    def diagnostics_callback(self, msg: DiagnosticArray):
        """Listen for configuration changes."""
        for status in msg.status:
            if status.name == 'navigation_controller':
                for kv in status.values:
                    if kv.key == 'current-configuration':
                        new_config = kv.value
                        if new_config != self.current_config:
                            self.current_config = new_config
                            self.adjust_component_power()

    # ───────────────────────────────────────────────
    def adjust_component_power(self):
        """Simulate component state changes."""
        old_perception = self.perception_active
        old_arm = self.arm_power_level
        
        if self.current_config == 'high_speed_config':
            self.perception_active = True
            self.arm_power_level = 'high'
            
        elif self.current_config == 'low_speed_config':
            self.perception_active = True
            self.arm_power_level = 'high'
            
        elif self.current_config == 'degraded_speed_config':
            self.perception_active = False
            self.arm_power_level = 'low'
        
        if old_perception != self.perception_active or old_arm != self.arm_power_level:
            self.get_logger().warn(
                f'🎛️ Config changed to {self.current_config}:\n'
                f'   Perception: {"ON" if self.perception_active else "OFF"} '
                f'(was {"ON" if old_perception else "OFF"})\n'
                f'   Arm: {self.arm_power_level.upper()} (was {old_arm.upper()})'
            )
            self.publish_component_states()

    # ───────────────────────────────────────────────
    def publish_component_states(self):
        """Publish simulated component states."""
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'battery_monitor'
        status.message = 'component state'
        status.level = DiagnosticStatus.OK
        
        kv_perception = KeyValue()
        kv_perception.key = 'perception-active'
        kv_perception.value = 'true' if self.perception_active else 'false'
        status.values.append(kv_perception)
        
        kv_arm = KeyValue()
        kv_arm.key = 'arm-power-level'
        kv_arm.value = self.arm_power_level
        status.values.append(kv_arm)
        
        diag_msg.status.append(status)
        self.diagnostics_pub.publish(diag_msg)

    # ───────────────────────────────────────────────
    def recharge_callback(self, msg: Bool):
        self.recharging = msg.data
        if self.recharging:
            self.recharge_complete = False
            self.get_logger().info("⚡ Recharging started...")
        else:
            self.get_logger().info("🔌 Recharging stopped.")

    # ───────────────────────────────────────────────
    def update_battery(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0:
            return

        distance = self.get_distance_traveled()

        if self.recharging:
            self.battery_level = min(1.0, self.battery_level + self.recharge_rate * dt)
            if self.battery_level >= 0.99 and not self.recharge_complete:
                self.recharge_complete = True
                self.recharging = False
                msg_done = Bool()
                msg_done.data = True
                self.recharge_complete_pub.publish(msg_done)
                self.get_logger().info("✅ Recharge complete! Battery full.")
            self.publish_battery_state(distance, 0.0, 0.0, 0.0)
            return

        # ═══════════════════════════════════════════════════════
        # ✅ PART 1: Base idle drain (just computers/sensors running)
        # ═══════════════════════════════════════════════════════
        idle_drain = self.base_idle_drain * dt / 100.0  # % per dt
        
        # ═══════════════════════════════════════════════════════
        # ✅ PART 2: Movement drain (motors, actual locomotion)
        # ═══════════════════════════════════════════════════════
        speed = distance / dt if dt > 0 else 0.0
        movement_drain = 0.0
        
        if speed > 0.01:  # Only drain from movement if actually moving
            # Base movement drain depends on configuration
            if self.current_config == 'high_speed_config':
                base_movement_rate = self.movement_drain_high
            elif self.current_config == 'low_speed_config':
                base_movement_rate = self.movement_drain_low
            else:  # degraded
                base_movement_rate = self.movement_drain_degraded
            
            # Scale by actual speed
            speed_factor = 1.0 + (self.speed_multiplier * speed)
            movement_drain = (base_movement_rate * dt / 100.0) * speed_factor
        
        # ═══════════════════════════════════════════════════════
        # ✅ PART 3: Component drain (perception, arm)
        # ═══════════════════════════════════════════════════════
        component_drain = 0.0
        
        if self.perception_active:
            component_drain += 0.08 * (dt / 100.0)
        
        if self.arm_power_level == 'high':
            component_drain += 0.12 * (dt / 100.0)
        elif self.arm_power_level == 'low':
            component_drain += 0.04 * (dt / 100.0)
        
        # ═══════════════════════════════════════════════════════
        # ✅ Total drain
        # ═══════════════════════════════════════════════════════
        total_drain = idle_drain + movement_drain + component_drain
        
        self.battery_level = max(0.0, self.battery_level - total_drain)
        self.publish_battery_state(distance, idle_drain, movement_drain, component_drain)

    def get_distance_traveled(self):
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            if self.prev_pose is None:
                self.prev_pose = (x, y)
                return 0.0
            dx, dy = x - self.prev_pose[0], y - self.prev_pose[1]
            self.prev_pose = (x, y)
            return math.sqrt(dx * dx + dy * dy)
        except LookupException:
            return 0.0
        except Exception as e:
            self.get_logger().warn(f"TF lookup error: {e}")
            return 0.0

    def publish_battery_state(self, distance: float, idle_drain: float, movement_drain: float, component_drain: float):
        msg = BatteryState()
        msg.voltage = 12.0
        msg.percentage = self.battery_level
        msg.present = True
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.recharging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        self.batt_pub.publish(msg)

        pct = self.battery_level * 100.0
        cfg = self.current_config
        state = "⚡ Charging" if self.recharging else "🔋 Discharging"
        
        perception_str = "👁️ ON " if self.perception_active else "👁️ OFF"
        arm_str = f"🦾 {self.arm_power_level.upper()}"
        
        dt = self.publish_period
        speed = distance / dt if dt > 0 else 0.0
        total = idle_drain + movement_drain + component_drain
        
        if total > 0:
            drain_info = (f"(-{total*100:.3f}%: idle={idle_drain*100:.3f}% + "
                         f"move={movement_drain*100:.3f}% + comp={component_drain*100:.3f}%)")
        else:
            drain_info = ""
        
        self.get_logger().info(
            f"{state} | {pct:5.1f}% {drain_info} | "
            f"speed={speed:.2f}m/s Δ={distance:.3f}m | "
            f"{cfg} | {perception_str} | {arm_str}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()