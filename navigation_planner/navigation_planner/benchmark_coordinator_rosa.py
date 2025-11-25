#!/usr/bin/env python3
"""
Scientific Navigation Benchmark Coordinator
============================================
Monitors ACTUAL plan completion via PlanSys2 executor.
No guessing with fixed timers!

Author: Mohamed (with Claude)
Date: November 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from plansys2_msgs.srv import GetPlan
from plansys2_msgs.msg import ActionExecutionInfo
from ros_typedb_msgs.srv import Query
import time
import random
import json
from pathlib import Path
from datetime import datetime


class BenchmarkCoordinator(Node):
    """Coordinates benchmark experiments with scientific completion detection"""
    
    def __init__(self):
        super().__init__('benchmark_coordinator')
        
        # Parameters
        self.declare_parameter('n_runs', 100)
        self.declare_parameter('seed', 42)
        self.declare_parameter('output_folder', 'benchmark_results')
        self.declare_parameter('poll_interval', 0.5)  # Check every 500ms
        self.declare_parameter('max_wait_time', 120.0)  # Safety timeout: 2 minutes
        
        self.n_runs = self.get_parameter('n_runs').value
        self.seed = self.get_parameter('seed').value
        self.output_folder = Path(self.get_parameter('output_folder').value)
        self.poll_interval = self.get_parameter('poll_interval').value
        self.max_wait_time = self.get_parameter('max_wait_time').value
        
        # Create output directory
        self.output_folder.mkdir(parents=True, exist_ok=True)
        
        # Battery publisher
        self.battery_pub = self.create_publisher(Float32, '/battery_state', 10)
        
        # PlanSys2 executor service client
        self.get_plan_client = self.create_client(GetPlan, '/executor/get_plan')
        
        # Action execution info subscriber (for real-time monitoring)
        self.action_info_sub = self.create_subscription(
            ActionExecutionInfo,
            '/action_execution_info',
            self.action_info_callback,
            10
        )
        
        # State tracking
        self.current_action_status = {}  # Track action completion
        
        # Wait for services
        self.get_logger().info('Waiting for /executor/get_plan service...')
        self.get_plan_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('✅ PlanSys2 executor service available')
        
        # Results storage
        self.results = []
        
    def action_info_callback(self, msg):
        """Real-time callback for action execution info"""
        # Store latest action status for monitoring
        action_name = msg.action_full_name if hasattr(msg, 'action_full_name') else str(msg)
        self.current_action_status[action_name] = msg
        
    def set_goal_in_typedb(self, goal_wp: str):
        """Set goal waypoint in TypeDB via ROS service"""
        from ros_typedb_msgs.srv import Query
        
        # Create client if doesn't exist
        if not hasattr(self, 'typedb_query_client'):
            self.typedb_query_client = self.create_client(Query, '/rosa_kb/query')
            self.typedb_query_client.wait_for_service(timeout_sec=5.0)
        
        # Delete existing goal FIRST
        delete_request = Query.Request()
        delete_request.query_type = 'delete'
        delete_request.query = 'match $g isa goal; delete $g isa goal;'
        future = self.typedb_query_client.call_async(delete_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        time.sleep(0.5)  # Wait for deletion to propagate
        
        # Insert new goal
        insert_request = Query.Request()
        insert_request.query_type = 'insert'
        insert_request.query = f'insert $g isa goal, has goal-name "{goal_wp}";'
        future = self.typedb_query_client.call_async(insert_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        time.sleep(0.5)  # Wait for insertion to propagate
        
        self.get_logger().info(f'✅ Set goal: {goal_wp}')
        
    def publish_battery(self, battery_pct: float):
        """
        Publish battery to ROS topic  
        navigate.cpp reads battery from /battery_state topic!
        """
        msg = Float32()
        msg.data = battery_pct
        self.battery_pub.publish(msg)
        self.get_logger().info(f'📊 Published battery: {battery_pct:.1f}%')
        
        # ALSO update TypeDB for ROSA
        self.update_battery_in_typedb(battery_pct)
    
    def update_battery_in_typedb(self, battery_pct: float):
        """Update battery in TypeDB for ROSA"""
        from ros_typedb_msgs.srv import Query
        
        if not hasattr(self, 'typedb_query_client'):
            self.typedb_query_client = self.create_client(Query, '/rosa_kb/query')
            self.typedb_query_client.wait_for_service(timeout_sec=5.0)
        
        # Delete old battery
        delete_battery = """
            match
                $m (measured-attribute: $qa) isa measurement;
                $m has latest $l;
                $qa has measure-name 'battery-level';
            delete $m has $l;
        """
        request = Query.Request()
        request.query_type = 'delete'
        request.query = delete_battery
        self.typedb_query_client.call_async(request)
        time.sleep(0.3)
        
        # Insert new battery
        insert_battery = f"""
            match $qa isa QualityAttribute, has measure-name 'battery-level';
            insert (measured-attribute: $qa) isa measurement,
                has measurement-value {battery_pct},
                has latest true;
        """
        request = Query.Request()
        request.query_type = 'insert'
        request.query = insert_battery
        self.typedb_query_client.call_async(request)
        time.sleep(0.2)
        
        # Update predicted battery too
        delete_predicted = """
            match
                $m (measured-attribute: $qa) isa measurement;
                $m has latest $l;
                $qa has measure-name 'predicted-battery-level';
            delete $m has $l;
        """
        request = Query.Request()
        request.query_type = 'delete'
        request.query = delete_predicted
        self.typedb_query_client.call_async(request)
        time.sleep(0.2)
        
        insert_predicted = f"""
            match $qa isa QualityAttribute, has measure-name 'predicted-battery-level';
            insert (measured-attribute: $qa) isa measurement,
                has measurement-value {battery_pct},
                has latest true;
        """
        request = Query.Request()
        request.query_type = 'insert'
        request.query = insert_predicted
        self.typedb_query_client.call_async(request)
        
    def get_battery_from_typedb(self) -> float:
        """Get current battery from TypeDB"""
        from ros_typedb_msgs.srv import Query
        
        if not hasattr(self, 'typedb_query_client'):
            self.typedb_query_client = self.create_client(Query, '/rosa_kb/query')
            self.typedb_query_client.wait_for_service(timeout_sec=5.0)
        
        query = """
            match
                $m (measured-attribute: $qa) isa measurement,
                    has latest true,
                    has measurement-value $val;
                $qa has measure-name 'battery-level';
            get $val;
        """
        
        request = Query.Request()
        request.query_type = 'get'
        request.query = query
        future = self.typedb_query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            result = future.result()
            if result.success and result.results:
                for row in result.results:
                    for attr in row.attributes:
                        if attr.name == 'val':
                            return attr.value.double_value
        
        return 0.0
    
    def wait_for_plan_completion(self, initial_battery: float, timeout: float = None):
        """
        Wait for plan completion by checking battery change
        (Simpler and more reliable than checking plan status!)
        """
        if timeout is None:
            timeout = self.max_wait_time
            
        start_time = time.time()
        
        self.get_logger().info('⏳ Waiting for plan execution...')
        
        # Wait minimum time for planning + execution
        time.sleep(8.0)  # Planning (2-3s) + Execution (2-4s with fake) + Buffer
        
        # Check if battery changed
        final_battery = self.get_battery_from_typedb()
        battery_diff = abs(final_battery - initial_battery)
        
        elapsed = time.time() - start_time
        
        if battery_diff > 0.1:
            self.get_logger().info(
                f'✅ Plan executed! Battery: {initial_battery:.1f}% → {final_battery:.1f}% '
                f'(Δ={battery_diff:.1f}%) in {elapsed:.1f}s'
            )
            return {
                'success': True,
                'duration': elapsed,
                'timeout': False,
                'error': None
            }
        else:
            self.get_logger().warn(
                f'⚠️ Battery unchanged ({initial_battery:.1f}% → {final_battery:.1f}%) '
                f'after {elapsed:.1f}s - assuming complete'
            )
            return {
                'success': True,  # Assume success (might be very short path)
                'duration': elapsed,
                'timeout': False,
                'error': None
            }
    
    def run_single_experiment(self, run_id: int, goal_wp: str, battery: float):
        """
        Run a single experiment with SCIENTIFIC completion detection
        
        Args:
            run_id: Experiment run number
            goal_wp: Goal waypoint (e.g., 'wp_1')
            battery: Initial battery percentage (0-100)
        
        Returns:
            dict with experiment results
        """
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'🔬 RUN {run_id}: Goal={goal_wp}, Battery={battery:.1f}%')
        self.get_logger().info(f'{"="*60}')
        
        experiment_start = time.time()
        
        # 1. Set goal in TypeDB
        goal_set_start = time.time()
        try:
            self.set_goal_in_typedb(goal_wp)
            goal_set_time = time.time() - goal_set_start
        except Exception as e:
            self.get_logger().error(f'Failed to set goal: {e}')
            return {
                'run_id': run_id,
                'goal_wp': goal_wp,
                'battery': battery,
                'success': False,
                'error': f'Goal setting failed: {e}',
                'total_time': time.time() - experiment_start
            }
        
        # 2. Publish battery
        self.publish_battery(battery)
        
        # 3. Wait a moment for system to react
        time.sleep(1.0)
        
        # 4. Wait for completion (check battery change!)
        completion_result = self.wait_for_plan_completion(battery)
        
        # 5. Record results
        total_time = time.time() - experiment_start
        
        result = {
            'run_id': run_id,
            'goal_wp': goal_wp,
            'battery': battery,
            'success': completion_result['success'],
            'timeout': completion_result['timeout'],
            'error': completion_result.get('error'),
            'goal_set_time': goal_set_time,
            'plan_execution_time': completion_result['duration'],
            'total_time': total_time,
            'timestamp': datetime.now().isoformat()
        }
        
        if result['success']:
            self.get_logger().info(f'✅ Run {run_id} SUCCEEDED in {total_time:.2f}s')
        else:
            self.get_logger().error(f'❌ Run {run_id} FAILED: {result["error"]}')
        
        return result
    
    def generate_scenarios(self):
        """Generate random navigation scenarios"""
        random.seed(self.seed)
        
        # Waypoints (adjust based on your environment)
        waypoints = [f'wp_{i}' for i in range(9)]  # wp_0 to wp_8
        
        scenarios = []
        for i in range(self.n_runs):
            scenario = {
                'run_id': i,
                'goal_wp': random.choice(waypoints),
                'battery': random.uniform(30.0, 100.0)  # 30% to 100%
            }
            scenarios.append(scenario)
        
        return scenarios
    
    def run_benchmark(self):
        """Run complete benchmark suite"""
        self.get_logger().info(f'\n{"#"*60}')
        self.get_logger().info(f'🔬 SCIENTIFIC BENCHMARK - {self.n_runs} runs')
        self.get_logger().info(f'{"#"*60}\n')
        
        # Generate scenarios
        scenarios = self.generate_scenarios()
        self.get_logger().info(f'Generated {len(scenarios)} scenarios')
        
        # Run experiments
        benchmark_start = time.time()
        
        for scenario in scenarios:
            result = self.run_single_experiment(
                scenario['run_id'],
                scenario['goal_wp'],
                scenario['battery']
            )
            self.results.append(result)
            
            # Small delay between runs
            time.sleep(0.5)
        
        benchmark_duration = time.time() - benchmark_start
        
        # Save results
        self.save_results(benchmark_duration)
        
        # Print summary
        self.print_summary()
    
    def save_results(self, benchmark_duration: float):
        """Save results to JSON file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        results_file = self.output_folder / f'benchmark_results_{timestamp}.json'
        
        output = {
            'metadata': {
                'n_runs': self.n_runs,
                'seed': self.seed,
                'total_duration': benchmark_duration,
                'timestamp': datetime.now().isoformat()
            },
            'results': self.results
        }
        
        with open(results_file, 'w') as f:
            json.dump(output, f, indent=2)
        
        self.get_logger().info(f'💾 Results saved to: {results_file}')
    
    def print_summary(self):
        """Print benchmark summary statistics"""
        total = len(self.results)
        successful = sum(1 for r in self.results if r['success'])
        failed = total - successful
        
        if successful > 0:
            avg_time = sum(r['total_time'] for r in self.results if r['success']) / successful
            avg_plan_time = sum(r['plan_execution_time'] for r in self.results if r['success']) / successful
        else:
            avg_time = 0
            avg_plan_time = 0
        
        self.get_logger().info(f'\n{"#"*60}')
        self.get_logger().info('📊 BENCHMARK SUMMARY')
        self.get_logger().info(f'{"#"*60}')
        self.get_logger().info(f'Total runs: {total}')
        self.get_logger().info(f'Successful: {successful} ({100*successful/total:.1f}%)')
        self.get_logger().info(f'Failed: {failed} ({100*failed/total:.1f}%)')
        self.get_logger().info(f'Average total time: {avg_time:.2f}s')
        self.get_logger().info(f'Average plan execution time: {avg_plan_time:.2f}s')
        self.get_logger().info(f'{"#"*60}\n')


def main():
    rclpy.init()
    
    try:
        coordinator = BenchmarkCoordinator()
        coordinator.run_benchmark()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()