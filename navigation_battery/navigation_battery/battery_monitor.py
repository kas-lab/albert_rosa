import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from ros_typedb_msgs.srv import Query

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.declare_parameter('robot_name', 'albert')
        self.declare_parameter('measure_name', 'battery-level')

        self.robot_name = self.get_parameter('robot_name').value
        self.measure_name = self.get_parameter('measure_name').value
        self.query_cli = self.create_client(Query, '/rosa_kb/query')
        self.sub = self.create_subscription(
            BatteryState, '/battery_state', self.callback, 10)

        self.get_logger().info('Battery monitor active')

    def callback(self, msg):
        percentage = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage

     
        delete_query = (
            'match '
            '$m (measured-attribute:$b) isa measurement; '
            '$b isa QualityAttribute, has measure-name "battery-level"; '
            'delete $m isa measurement;'
        )

        insert_query = (
            f'match '
            f'$b isa QualityAttribute, has measure-name "battery-level"; '
            f'insert (measured-attribute:$b) isa measurement, '
            f'has measurement-value {percentage}, '
            f'has latest true;'
        )


        if not self.query_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('TypeDB query service not available')
            return

        del_req = Query.Request()
        del_req.query_type = "delete"
        del_req.query = delete_query
        self.query_cli.call_async(del_req)

        ins_req = Query.Request()
        ins_req.query_type = "insert"
        ins_req.query = insert_query
        self.query_cli.call_async(ins_req)

        self.get_logger().info(f'Battery {percentage:.2f}% → KB updated')



def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
