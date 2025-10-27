# file: battery_monitor_proactive_tf.py
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from ros_typedb_msgs.srv import Query
from tf2_ros import Buffer, TransformListener, LookupException

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_proactive_tf')

        # Parameters
        self.declare_parameter('robot_name', 'albert')
        self.declare_parameter('measure_name', 'battery-level')
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('discharge_time_high', 180.0)
        self.declare_parameter('discharge_time_low', 300.0)
        self.declare_parameter('initial_level', 1.0)
        self.declare_parameter('distance_scale', 2.0)  # energy cost per meter (tunable)

        # Internal state
        self.robot_name = self.get_parameter('robot_name').value
        self.measure_name = self.get_parameter('measure_name').value
        self.battery_level = float(self.get_parameter('initial_level').value)
        self.discharge_time_high = float(self.get_parameter('discharge_time_high').value)
        self.discharge_time_low = float(self.get_parameter('discharge_time_low').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.distance_scale = float(self.get_parameter('distance_scale').value)

        self.current_config = 'high_speed_config'
        self.last_time = self.get_clock().now()
        self.prev_pose = None

        # ROS interfaces
        self.query_cli = self.create_client(Query, '/rosa_kb/query')
        self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)

        # TF buffer and listener to get robot motion
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to ROSA events
        self.event_sub = self.create_subscription(
            String, '/rosa_kb/events', self.rosa_event_callback, 10)

        # Periodic timer
        self.timer = self.create_timer(self.publish_period, self.update_battery)

        self.get_logger().info(
            f'BatteryMonitor started — initial level: {self.battery_level*100:.1f}% '
            f'| config={self.current_config}'
        )

    # ─────────────────────────────────────────────────────────────
    # Event callback from ROSA KB
    # ─────────────────────────────────────────────────────────────
    def rosa_event_callback(self, msg: String):
        if "reconfiguration" in msg.data:
            self.get_logger().info(f'🔄 ROSA event detected: {msg.data}')
            # Query KB after short delay (ensure KB updated)
            self.create_timer(1.0, self.query_selected_config_once)

    def query_selected_config_once(self):
        self.query_selected_config()
        # destroy timer after one-shot
        self.destroy_timer(self.query_selected_config_once)

    def query_selected_config(self):
        if not self.query_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning('TypeDB query service not available')
            return

        query = (
            'match $cc isa component-configuration, '
            'has is-selected true, '
            'has component-configuration-name $name; '
            'fetch $name;'
        )

        req = Query.Request()
        req.query_type = "fetch"
        req.query = query
        
        future = self.query_cli.call_async(req)
        future.add_done_callback(self.handle_config_query_response)

    def handle_config_query_response(self, future):
        try:
            response = future.result()
            if response.success and response.results:
                for result in response.results:
                    for attr in result.attributes:
                        if attr.name == 'name':
                            new_config = attr.value.string_value
                            if new_config != self.current_config:
                                old_config = self.current_config
                                self.current_config = new_config
                                self.get_logger().info(
                                    f'✅ Config switched: {old_config} → {new_config}')
                            break
        except Exception as e:
            self.get_logger().error(f'Error querying config: {e}')

    # ─────────────────────────────────────────────────────────────
    # Battery model update
    # ─────────────────────────────────────────────────────────────
    def update_battery(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0:
            return

        # Baseline drain depends on configuration
        discharge_time = (
            self.discharge_time_low
            if self.current_config == 'low_speed_config'
            else self.discharge_time_high
        )
        base_drain = dt / discharge_time

        # Get distance moved from /tf
        distance = self.get_distance_traveled()

        # Idle factor = how much power the robot uses even when stationary (10–20%)
        idle_factor = 0.15

        # Scale motion effect (adds extra drain when moving)
        motion_factor = idle_factor + self.distance_scale * distance

        # Apply combined discharge (baseline + motion-based)
        self.battery_level -= base_drain * motion_factor
        if self.battery_level < 0.0:
            self.battery_level = 0.0

        percentage = self.battery_level * 100.0

        # Optional debug to see how TF affects it
        self.get_logger().info(
            f"🔋 {percentage:.1f}% | Δ={distance:.3f} m | motion_factor={motion_factor:.3f} | cfg={self.current_config}"
        )

        self.publish_battery_state(percentage)
        self.update_typedb(percentage)


    def get_distance_traveled(self):
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

    # ─────────────────────────────────────────────────────────────
    # ROS publishing and TypeDB updates
    # ─────────────────────────────────────────────────────────────
    def publish_battery_state(self, percentage):
        msg = BatteryState()
        msg.voltage = 12.0
        msg.percentage = max(0.0, min(1.0, percentage / 100.0))
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.batt_pub.publish(msg)

    def update_typedb(self, percentage):
        if not self.query_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning('TypeDB query service not available')
            return

        delete_query = (
            'match $m (measured-attribute:$b) isa measurement; '
            '$b isa QualityAttribute, has measure-name "battery-level"; '
            'delete $m isa measurement;'
        )

        insert_query = (
            f'match $b isa QualityAttribute, has measure-name "battery-level"; '
            f'insert (measured-attribute:$b) isa measurement, '
            f'has measurement-value {percentage}, has latest true;'
        )

        del_req = Query.Request()
        del_req.query_type = "delete"
        del_req.query = delete_query
        self.query_cli.call_async(del_req)

        ins_req = Query.Request()
        ins_req.query_type = "insert"
        ins_req.query = insert_query
        self.query_cli.call_async(ins_req)

        self.get_logger().info(f'🔋 Battery {percentage:.1f}% ({self.current_config}) → KB updated')

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
