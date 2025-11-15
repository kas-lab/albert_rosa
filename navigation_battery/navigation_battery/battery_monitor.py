#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, LookupException


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_proactive_tf')

        # ─────────────── Parameters ───────────────
        self.declare_parameter('discharge_time_high', 180.0)
        self.declare_parameter('discharge_time_low', 300.0)
        self.declare_parameter('initial_level', 1.0)
        self.declare_parameter('distance_scale', 2.0)
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('recharge_rate', 0.01)  # % per second

        # ─────────────── Internal State ───────────────
        self.battery_level = float(self.get_parameter('initial_level').value)
        self.discharge_time_high = float(self.get_parameter('discharge_time_high').value)
        self.discharge_time_low = float(self.get_parameter('discharge_time_low').value)
        self.distance_scale = float(self.get_parameter('distance_scale').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.recharge_rate = float(self.get_parameter('recharge_rate').value)
        self.current_config = 'high_speed_config'
        self.recharging = False
        self.recharge_complete = False
        self.prev_pose = None
        self.last_time = self.get_clock().now()

        # ─────────────── ROS Interfaces ───────────────
        self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.recharge_complete_pub = self.create_publisher(Bool, '/battery_monitor/recharge_complete', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.recharge_sub = self.create_subscription(
            Bool, '/battery_monitor/recharge', self.recharge_callback, 10
        )

        # ─────────────── Periodic Timer ───────────────
        self.timer = self.create_timer(self.publish_period, self.update_battery)
        self.get_logger().info(f'BatteryMonitor running | initial={self.battery_level*100:.1f}%')

    # ───────────────────────────────────────────────
    def recharge_callback(self, msg: Bool):
        """Enable or disable recharging."""
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

        # ─────────────── RECHARGING ───────────────
        if self.recharging:
            self.battery_level = min(1.0, self.battery_level + self.recharge_rate * dt)
            pct = self.battery_level * 100.0
            if self.battery_level >= 0.99 and not self.recharge_complete:
                self.recharge_complete = True
                self.recharging = False
                msg_done = Bool()
                msg_done.data = True
                self.recharge_complete_pub.publish(msg_done)
                self.get_logger().info("✅ Recharge complete! Battery full.")
            self.publish_battery_state(distance)
            return

        # ─────────────── DISCHARGING ───────────────
        discharge_time = (
            self.discharge_time_low
            if self.current_config == 'low_speed_config'
            else self.discharge_time_high
        )
        base_drain = dt / discharge_time
        motion_factor = 0.15 + self.distance_scale * distance
        self.battery_level = max(0.0, self.battery_level - base_drain * motion_factor)
        self.publish_battery_state(distance)

    # ───────────────────────────────────────────────
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

    # ───────────────────────────────────────────────
    def publish_battery_state(self, distance: float):
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
        self.get_logger().info(f"{state} | {pct:5.1f}% | Δ={distance:0.3f} m | cfg={cfg}")


# ───────────────────────────────────────────────
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
