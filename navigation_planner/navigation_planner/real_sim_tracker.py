#!/usr/bin/env python3
"""
Real Simulation Metrics Monitor - TF-based distance tracking with full parameter discovery
==========================================================================================
Uses TF transform (odom->base_link) exactly like battery_monitor.py
Auto-discovers and tracks Nav2 controller parameters using ROS2 services
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters, ListParameters
import json
import math
import subprocess
from datetime import datetime
from pathlib import Path
from tf2_ros import Buffer, TransformListener, LookupException


class RealSimMonitor(Node):
    def __init__(self):
        super().__init__('real_sim_monitor')
        
        # TF setup for distance tracking (like battery_monitor)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Run tracking
        self.runs = []
        self.current_run = {
            'run_number': 1,
            'start_time': None,
            'end_time': None,
            'start_battery': None,
            'end_battery': None,
            'battery_used': None,
            'adaptations': 0,
            'config_changes': [],
            'adaptation_events': [],
            'move_actions': 0,
            'recharge_count': 0,
            'distance_traveled': 0.0,
            'success': True,
            'config_usage': {},
            'move_details': [],
            'perception_changes': 0,
            'arm_changes': 0,
            'component_events': [],
            'waypoints_visited': [],
            'controller_params': {},
            'navigation_events': [],
            'param_snapshots': [],
            'battery_timeline': [],  # Time-series battery data for plotting
        }
        
        # State tracking
        self.last_config = None
        self.config_start_time = self.get_clock().now()
        self.last_perception = None
        self.last_arm_level = None
        self.prev_tf_pose = None
        self.current_waypoint = None
        self.mission_started = False
        self.tf_available = False
        
        # Subscribe to battery
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        # Subscribe to recharge events
        self.recharge_sub = self.create_subscription(
            Bool,
            '/battery_monitor/recharge_complete',
            self.recharge_complete_callback,
            10
        )
        
        # Subscribe to ROSA monitoring
        self.monitoring_sub = self.create_subscription(
            DiagnosticArray,
            '/navigation/monitoring',
            self.monitoring_callback,
            10
        )
        
        # Subscribe to action events
        self.action_events_sub = self.create_subscription(
            String,
            '/action_events',
            self.action_event_callback,
            10
        )
        
        # Manual run control
        self.run_start_sub = self.create_subscription(
            Int32,
            '/metrics/run_start',
            self.run_start_callback,
            10
        )
        
        self.run_end_sub = self.create_subscription(
            Int32,
            '/metrics/run_end',
            self.run_end_callback,
            10
        )
        
        # Accept run_stop as alias
        self.run_stop_sub = self.create_subscription(
            Int32,
            '/metrics/run_stop',
            self.run_end_callback,
            10
        )
        
        # Timers
        self.param_timer = self.create_timer(5.0, self.sample_parameters)
        self.distance_timer = self.create_timer(0.1, self.update_distance_from_tf)
        self.diag_timer = self.create_timer(10.0, self.check_tf_status)
        
        # Results directory
        self.results_dir = Path('real_sim_results')
        self.results_dir.mkdir(exist_ok=True)
        
        self.get_logger().info('🔬 Real Simulation Metrics Monitor ready!')
        self.get_logger().info('   Tracking: TF (odom->base_link), Nav2, ROSA, Battery')
        self.get_logger().info('   AUTO-START: Mission tracking begins on first move')
        self.get_logger().info('   To end run: ros2 topic pub -1 /metrics/run_end std_msgs/msg/Int32 "data: 1"')
    
    def update_distance_from_tf(self):
        """Track distance using TF (exactly like battery_monitor.py)"""
        if not self.mission_started:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            current_pose = (x, y)
            
            if not self.tf_available:
                self.tf_available = True
                self.get_logger().info(
                    f"📡 TF 'odom->base_link' ACTIVE! Initial: ({x:.2f}, {y:.2f})")
            
            if self.prev_tf_pose is not None:
                dx = current_pose[0] - self.prev_tf_pose[0]
                dy = current_pose[1] - self.prev_tf_pose[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > 0.001:
                    self.current_run['distance_traveled'] += distance
                    
                    total = self.current_run['distance_traveled']
                    if int(total) > int(total - distance):
                        self.get_logger().info(f"📏 Distance: {total:.2f}m")
            
            self.prev_tf_pose = current_pose
            
        except LookupException:
            pass
        except Exception as e:
            self.get_logger().debug(f'TF error: {e}')
    
    def check_tf_status(self):
        """Check if TF is working"""
        if self.mission_started and not self.tf_available:
            self.get_logger().warn(
                "⚠️ TF 'odom->base_link' not available! "
                "Check: ros2 run tf2_ros tf2_echo odom base_link")
    
    def sample_parameters(self):
        """Query Nav2 controller parameters using ros2param dump approach"""
        if not self.mission_started:
            return
        
        # First time: discover what parameters exist
        if not hasattr(self, 'discovered_params'):
            self.discover_controller_params()
            return
        
        # If discovery failed, try command-line fallback
        if not self.discovered_params:
            self.try_command_line_params()
            return
        
        # Query the discovered parameters via service
        self.query_discovered_params()
    
    def discover_controller_params(self):
        """Discover available controller parameters (like ros2 param list)"""
        if not hasattr(self, 'list_param_client'):
            self.list_param_client = self.create_client(
                ListParameters,
                '/controller_server/list_parameters'
            )
        
        if not self.list_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⚠️ Controller param service not available, trying command-line")
            self.discovered_params = []
            return
        
        req = ListParameters.Request()
        future = self.list_param_client.call_async(req)
        future.add_done_callback(self.handle_param_discovery)
    
    def handle_param_discovery(self, future):
        """Process discovered parameters"""
        try:
            response = future.result()
            
            # Focus on the most important parameters for ROSA configs
            important_keywords = [
                'max_vel_x', 'max_vel_y', 'max_vel_theta',
                'min_vel_x', 'min_vel_y', 'min_speed',
                'max_speed', 'acc_lim', 'decel_lim'
            ]
            
            important_params = [
                name for name in response.result.names
                if any(keyword in name for keyword in important_keywords)
            ]
            
            self.discovered_params = important_params
            
            if important_params:
                self.get_logger().info(
                    f"📊 Discovered {len(important_params)} controller params:")
                for p in important_params:
                    self.get_logger().info(f"   - {p}")
            else:
                self.get_logger().warn("⚠️ No velocity/speed parameters found!")
                
        except Exception as e:
            self.get_logger().error(f"Parameter discovery failed: {e}")
            self.discovered_params = []
    
    def query_discovered_params(self):
        """Query the discovered parameters"""
        if not self.discovered_params:
            return
        
        if not hasattr(self, 'get_param_client'):
            self.get_param_client = self.create_client(
                GetParameters,
                '/controller_server/get_parameters'
            )
        
        if not self.get_param_client.service_is_ready():
            return
        
        req = GetParameters.Request()
        req.names = self.discovered_params
        
        future = self.get_param_client.call_async(req)
        future.add_done_callback(self.handle_param_values)
    
    def handle_param_values(self, future):
        """Process parameter values"""
        try:
            response = future.result()
            
            params = {}
            for i, param_name in enumerate(self.discovered_params):
                if i >= len(response.values):
                    break
                
                value = response.values[i]
                
                # Extract actual value based on type
                if value.type == 3:  # PARAMETER_DOUBLE
                    params[param_name] = value.double_value
                elif value.type == 2:  # PARAMETER_INTEGER
                    params[param_name] = float(value.integer_value)
            
            if params:
                snapshot = {
                    'timestamp': datetime.now().isoformat(),
                    'config': self.last_config,
                    'controller_params': params
                }
                
                self.current_run['param_snapshots'].append(snapshot)
                
                # Log the most important params (velocities and accelerations)
                key_params = {}
                for k, v in params.items():
                    # Shorten param names for readability
                    short_name = k.replace('FollowPath.', '')
                    if any(x in short_name for x in ['max_vel', 'min_vel', 'acc_lim', 'max_speed', 'min_speed']):
                        key_params[short_name] = v
                
                if key_params:
                    self.get_logger().info(
                        f"📊 Controller @ {self.last_config}:")
                    for k, v in sorted(key_params.items()):
                        self.get_logger().info(f"   {k:20s} = {v:.3f}")
                    
        except Exception as e:
            self.get_logger().debug(f'Param query error: {e}')
    
    def try_command_line_params(self):
        """Fallback: Use ros2 param list + get via command line"""
        try:
            # List all parameters
            result = subprocess.run(
                ['ros2', 'param', 'list', '/controller_server'],
                capture_output=True,
                text=True,
                timeout=2.0
            )
            
            if result.returncode != 0:
                return
            
            # Find important parameters (same as service-based discovery)
            all_params = result.stdout.strip().split('\n')
            important_keywords = [
                'max_vel_x', 'max_vel_y', 'max_vel_theta',
                'min_vel_x', 'min_vel_y', 'min_speed',
                'max_speed', 'acc_lim', 'decel_lim'
            ]
            
            important_params = [
                p.strip() for p in all_params
                if any(kw in p for kw in important_keywords)
            ]
            
            if not important_params:
                self.get_logger().warn("⚠️ No important params found via command line")
                self.discovered_params = []
                return
            
            self.get_logger().info(
                f"📊 Discovered {len(important_params)} params via CLI:")
            for p in important_params:
                self.get_logger().info(f"   - {p}")
            
            # Get their values
            params = {}
            for param in important_params:
                try:
                    result = subprocess.run(
                        ['ros2', 'param', 'get', '/controller_server', param],
                        capture_output=True,
                        text=True,
                        timeout=1.0
                    )
                    
                    if result.returncode == 0:
                        output = result.stdout.strip()
                        if 'value is:' in output:
                            value_str = output.split('value is:')[1].strip()
                            try:
                                params[param] = float(value_str)
                            except ValueError:
                                pass
                                
                except subprocess.TimeoutExpired:
                    continue
            
            if params:
                snapshot = {
                    'timestamp': datetime.now().isoformat(),
                    'config': self.last_config,
                    'controller_params': params
                }
                
                self.current_run['param_snapshots'].append(snapshot)
                
                # Log with same formatting as service method
                key_params = {}
                for k, v in params.items():
                    short_name = k.replace('FollowPath.', '')
                    if any(x in short_name for x in ['max_vel', 'min_vel', 'acc_lim', 'max_speed', 'min_speed']):
                        key_params[short_name] = v
                
                if key_params:
                    self.get_logger().info(
                        f"📊 Controller (CLI) @ {self.last_config}:")
                    for k, v in sorted(key_params.items()):
                        self.get_logger().info(f"   {k:20s} = {v:.3f}")
                
                # Cache discovered params for next time
                self.discovered_params = important_params
            
        except Exception as e:
            self.get_logger().debug(f'Command-line discovery failed: {e}')
            self.discovered_params = []
    
    def battery_callback(self, msg: BatteryState):
        """Track battery and record time-series data"""
        if msg.percentage is None:
            return
        
        battery = msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage
        
        if not (0 <= battery <= 100):
            return
        
        if self.current_run['start_battery'] is None and self.mission_started:
            self.current_run['start_battery'] = battery
            self.get_logger().info(f"🔋 Mission battery: {battery:.1f}%")
        
        self.current_run['end_battery'] = battery
        
        # Record time-series data point
        if self.mission_started and self.current_run['start_time']:
            start_time = datetime.fromisoformat(self.current_run['start_time'])
            elapsed = (datetime.now() - start_time).total_seconds()
            
            battery_snapshot = {
                'timestamp': elapsed,
                'battery': battery,
                'distance': self.current_run['distance_traveled'],
                'config': self.last_config,
                'perception': self.last_perception,
                'arm': self.last_arm_level,
                'is_charging': msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            }
            
            self.current_run['battery_timeline'].append(battery_snapshot)
        
        if battery < 1.0 and not self.current_run.get('failed', False):
            self.current_run['success'] = False
            self.current_run['failed'] = True
            self.get_logger().error(f'❌ Run {self.current_run["run_number"]}: Battery depleted!')
    
    def recharge_complete_callback(self, msg: Bool):
        if msg.data and self.mission_started:
            self.current_run['recharge_count'] += 1
            
            # Mark recharge event in battery timeline
            if self.current_run['start_time']:
                start_time = datetime.fromisoformat(self.current_run['start_time'])
                elapsed = (datetime.now() - start_time).total_seconds()
                
                recharge_marker = {
                    'timestamp': elapsed,
                    'battery': self.current_run['end_battery'],
                    'distance': self.current_run['distance_traveled'],
                    'config': self.last_config,
                    'perception': self.last_perception,
                    'arm': self.last_arm_level,
                    'is_charging': False,
                    'recharge_event': self.current_run['recharge_count']  # Mark as recharge
                }
                
                self.current_run['battery_timeline'].append(recharge_marker)
            
            self.current_run['navigation_events'].append(
                f"Recharge #{self.current_run['recharge_count']}")
            self.get_logger().info(
                f"⚡ Recharge #{self.current_run['recharge_count']} completed")
    
    def monitoring_callback(self, msg: DiagnosticArray):
        """Track ROSA adaptations"""
        if not self.mission_started:
            return
        
        current_time = self.get_clock().now()
        
        for status in msg.status:
            if status.name == 'navigation_monitoring':
                for kv in status.values:
                    if kv.key == 'current-configuration':
                        config = kv.value
                        
                        if self.last_config is not None:
                            duration = (current_time - self.config_start_time).nanoseconds / 1e9
                            if self.last_config not in self.current_run['config_usage']:
                                self.current_run['config_usage'][self.last_config] = 0.0
                            self.current_run['config_usage'][self.last_config] += duration
                        
                        if self.last_config is not None and self.last_config != config:
                            self.current_run['config_changes'].append(config)
                            self.current_run['adaptations'] += 1
                            
                            reason = "unknown"
                            for kv2 in status.values:
                                if kv2.key == 'adaptation-reason':
                                    reason = kv2.value
                                    break
                            
                            event = f"{config} (reason: {reason})"
                            self.current_run['adaptation_events'].append(event)
                            
                            self.get_logger().info(
                                f"🔄 Adaptation {self.current_run['adaptations']}: "
                                f"{self.last_config} → {config} ({reason})")
                        
                        self.last_config = config
                        self.config_start_time = current_time
                    
                    elif kv.key == 'perception-active':
                        perception = kv.value.lower() == 'true'
                        
                        if self.last_perception is not None and perception != self.last_perception:
                            self.current_run['perception_changes'] += 1
                            state = 'ON' if perception else 'OFF'
                            event = f"Perception → {state}"
                            self.current_run['component_events'].append(event)
                            
                            self.get_logger().info(f"👁️ Perception → {state}")
                        
                        self.last_perception = perception
                    
                    elif kv.key == 'arm-power-level':
                        arm_level = kv.value
                        
                        if self.last_arm_level is not None and arm_level != self.last_arm_level:
                            self.current_run['arm_changes'] += 1
                            event = f"Arm → {arm_level.upper()}"
                            self.current_run['component_events'].append(event)
                            
                            self.get_logger().info(f"🦾 Arm → {arm_level.upper()}")
                        
                        self.last_arm_level = arm_level
    
    def action_event_callback(self, msg: String):
        """Track moves - SAME LOGIC AS benchmark_monitor.py"""
        parts = msg.data.split('|')
        event_type = parts[0]
        
        if event_type == "MOVE_START":
            from_wp = parts[1]
            to_wp = parts[2]
            config = parts[3]
            distance = float(parts[4]) if len(parts) > 4 else 5.0
            action_type = parts[5] if len(parts) > 5 else "move_unknown"
            
            if not self.mission_started:
                self.get_logger().warn("⚠️ Auto-starting mission from first move")
                self.mission_started = True
                self.current_run['start_time'] = datetime.now().isoformat()
            
            self.current_run['move_actions'] += 1
            self.current_waypoint = to_wp
            
            move_detail = {
                'number': self.current_run['move_actions'],
                'from': from_wp,
                'to': to_wp,
                'config': config,
                'distance': distance,
                'action_type': action_type,
                'timestamp': datetime.now().isoformat()
            }
            self.current_run['move_details'].append(move_detail)
            
            # Track waypoint visit on START (like benchmark_monitor tracks it)
            if to_wp not in self.current_run['waypoints_visited']:
                self.current_run['waypoints_visited'].append(to_wp)
            
            self.get_logger().info(
                f"🚗 Move #{self.current_run['move_actions']}: "
                f"{from_wp} → {to_wp} ({config}, {action_type}) [{distance:.2f}m]")
        
        elif event_type == "MOVE_END":
            from_wp = parts[1]
            to_wp = parts[2]
            # Just debug log like benchmark_monitor
            self.get_logger().debug(f"✅ Move completed: {from_wp} → {to_wp}")
        
        elif event_type == "RECHARGE_START":
            wp = parts[1] if len(parts) > 1 else "unknown"
            self.current_run['recharge_count'] += 1
            self.get_logger().info(f"⚡ Recharge #{self.current_run['recharge_count']} at {wp}")
        
        elif event_type == "RECHARGE_END":
            self.get_logger().debug("✅ Recharge completed")
    
    def run_start_callback(self, msg: Int32):
        """Manual run start"""
        run_num = msg.data
        
        if self.mission_started:
            self.get_logger().warn("Previous run active - finalizing")
            self.finalize_run()
        
        self.current_run = {
            'run_number': run_num,
            'start_time': datetime.now().isoformat(),
            'end_time': None,
            'start_battery': None,
            'end_battery': None,
            'battery_used': None,
            'adaptations': 0,
            'config_changes': [],
            'adaptation_events': [],
            'move_actions': 0,
            'recharge_count': 0,
            'distance_traveled': 0.0,
            'success': True,
            'config_usage': {},
            'move_details': [],
            'perception_changes': 0,
            'arm_changes': 0,
            'component_events': [],
            'waypoints_visited': [],
            'controller_params': {},
            'navigation_events': [],
            'param_snapshots': [],
            'battery_timeline': [],
        }
        
        self.last_config = None
        self.config_start_time = self.get_clock().now()
        self.last_perception = None
        self.last_arm_level = None
        self.prev_tf_pose = None
        self.mission_started = True
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🚀 STARTING RUN {run_num}")
        self.get_logger().info(f"{'='*60}\n")
    
    def run_end_callback(self, msg: Int32):
        """End run"""
        run_num = msg.data
        
        if not self.mission_started:
            self.get_logger().warn(f"Run {run_num} end but no active run")
            return
        
        self.finalize_run()
        self.save_results()
        self.mission_started = False
    
    def finalize_run(self):
        """Finalize run"""
        self.current_run['end_time'] = datetime.now().isoformat()
        
        start = datetime.fromisoformat(self.current_run['start_time'])
        end = datetime.fromisoformat(self.current_run['end_time'])
        duration = (end - start).total_seconds()
        self.current_run['duration_seconds'] = duration
        
        if self.last_config is not None:
            current_time = self.get_clock().now()
            final_duration = (current_time - self.config_start_time).nanoseconds / 1e9
            if self.last_config not in self.current_run['config_usage']:
                self.current_run['config_usage'][self.last_config] = 0.0
            self.current_run['config_usage'][self.last_config] += final_duration
        
        if self.current_run['start_battery'] is not None and \
           self.current_run['end_battery'] is not None:
            battery_used = self.current_run['start_battery'] - self.current_run['end_battery']
        else:
            battery_used = None
        
        self.current_run['battery_used'] = battery_used
        
        self.runs.append(self.current_run.copy())
        self.print_run_summary()
    
    def print_run_summary(self):
        """Print summary"""
        run = self.current_run
        success_emoji = "✅" if run['success'] else "❌"
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"{success_emoji} RUN {run['run_number']} COMPLETE")
        self.get_logger().info(f"{'='*60}")
        
        self.get_logger().info(f"⏱️  Duration: {run.get('duration_seconds', 0):.1f}s")
        
        if run['battery_used']:
            self.get_logger().info(
                f"🔋 Battery: {run['start_battery']:.1f}% → {run['end_battery']:.1f}% "
                f"(used {run['battery_used']:.1f}%)")
        
        self.get_logger().info(f"📏 Distance: {run['distance_traveled']:.2f}m")
        self.get_logger().info(f"🚗 Moves: {run['move_actions']}")
        self.get_logger().info(f"⚡ Recharges: {run['recharge_count']}")
        self.get_logger().info(f"📍 Waypoints: {len(run['waypoints_visited'])}")
        
        self.get_logger().info(f"🔄 Adaptations: {run['adaptations']}")
        for i, event in enumerate(run['adaptation_events'], 1):
            self.get_logger().info(f"   {i}. {event}")
        
        total_component = run.get('perception_changes', 0) + run.get('arm_changes', 0)
        if total_component > 0:
            self.get_logger().info(
                f"🔧 Component: {total_component} "
                f"(👁️ {run.get('perception_changes', 0)}, 🦾 {run.get('arm_changes', 0)})")
        
        self.get_logger().info("⚙️  Config usage:")
        for config, duration in run['config_usage'].items():
            pct = (duration / run.get('duration_seconds', 1)) * 100
            self.get_logger().info(f"   {config}: {duration:.1f}s ({pct:.1f}%)")
        
        self.get_logger().info(f"📊 Parameter snapshots: {len(run.get('param_snapshots', []))}")
        self.get_logger().info(f"📈 Battery data points: {len(run.get('battery_timeline', []))}")
        
        self.get_logger().info(f"{'='*60}\n")
    
    def save_results(self):
        """Save results"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = self.results_dir / f'real_sim_{timestamp}.json'
        
        summary = {
            'total_runs': len(self.runs),
            'timestamp': datetime.now().isoformat(),
            'mode': 'real_simulation',
            'runs': self.runs,
            'statistics': self.calculate_statistics()
        }
        
        with open(filepath, 'w') as f:
            json.dump(summary, f, indent=2)
        
        self.get_logger().info(f"💾 Saved: {filepath}")
    
    def calculate_statistics(self):
        """Calculate stats"""
        if not self.runs:
            return {}
        
        successful = [r for r in self.runs if r.get('success', True)]
        
        stats = {
            'total_runs': len(self.runs),
            'successful_runs': len(successful),
            'success_rate': len(successful) / len(self.runs) * 100,
            'avg_duration': sum(r.get('duration_seconds', 0) for r in self.runs) / len(self.runs),
            'avg_distance': sum(r['distance_traveled'] for r in self.runs) / len(self.runs),
            'total_distance': sum(r['distance_traveled'] for r in self.runs),
            'avg_adaptations': sum(r['adaptations'] for r in self.runs) / len(self.runs),
            'total_adaptations': sum(r['adaptations'] for r in self.runs),
            'avg_move_actions': sum(r['move_actions'] for r in self.runs) / len(self.runs),
            'avg_recharges': sum(r['recharge_count'] for r in self.runs) / len(self.runs),
        }
        
        battery_values = [r['battery_used'] for r in self.runs if r.get('battery_used')]
        if battery_values:
            stats['avg_battery_used'] = sum(battery_values) / len(battery_values)
            stats['min_battery_remaining'] = min(r['end_battery'] for r in self.runs if r.get('end_battery'))
            stats['max_battery_remaining'] = max(r['end_battery'] for r in self.runs if r.get('end_battery'))
        
        all_config_usage = {}
        for run in self.runs:
            for config, duration in run.get('config_usage', {}).items():
                if config not in all_config_usage:
                    all_config_usage[config] = 0.0
                all_config_usage[config] += duration
        
        total_time = sum(all_config_usage.values())
        if total_time > 0:
            stats['config_usage_percentage'] = {
                config: (duration / total_time * 100)
                for config, duration in all_config_usage.items()
            }
        
        # Parameter snapshots summary
        all_param_snapshots = []
        for run in self.runs:
            all_param_snapshots.extend(run.get('param_snapshots', []))
        
        if all_param_snapshots:
            stats['total_param_snapshots'] = len(all_param_snapshots)
            
            # Group by config to show typical parameters per config
            params_by_config = {}
            for snapshot in all_param_snapshots:
                config = snapshot.get('config', 'unknown')
                params = snapshot.get('controller_params', {})
                
                if config not in params_by_config:
                    params_by_config[config] = []
                params_by_config[config].append(params)
            
            # Calculate average params per config
            avg_params_by_config = {}
            for config, param_list in params_by_config.items():
                if not param_list:
                    continue
                
                # Get all param names
                all_keys = set()
                for p in param_list:
                    all_keys.update(p.keys())
                
                # Average each param
                avg_params = {}
                for key in sorted(all_keys):
                    values = [p[key] for p in param_list if key in p]
                    if values:
                        avg_params[key] = sum(values) / len(values)
                
                avg_params_by_config[config] = avg_params
            
            stats['avg_controller_params_by_config'] = avg_params_by_config
            
            # Also track min/max to see ranges
            min_max_params = {}
            for config, param_list in params_by_config.items():
                if not param_list:
                    continue
                    
                all_keys = set()
                for p in param_list:
                    all_keys.update(p.keys())
                
                ranges = {}
                for key in sorted(all_keys):
                    values = [p[key] for p in param_list if key in p]
                    if values:
                        ranges[key] = {
                            'min': min(values),
                            'max': max(values),
                            'avg': sum(values) / len(values)
                        }
                
                min_max_params[config] = ranges
            
            stats['param_ranges_by_config'] = min_max_params
        
        # Battery timeline statistics
        total_battery_points = sum(len(run.get('battery_timeline', [])) for run in self.runs)
        if total_battery_points > 0:
            stats['total_battery_data_points'] = total_battery_points
            stats['avg_battery_points_per_run'] = total_battery_points / len(self.runs)
            
            # Calculate average drain rates by config
            drain_by_config = {}
            for run in self.runs:
                timeline = run.get('battery_timeline', [])
                if len(timeline) < 2:
                    continue
                
                for i in range(1, len(timeline)):
                    prev = timeline[i-1]
                    curr = timeline[i]
                    
                    # Skip recharge events
                    if curr.get('recharge_event') or prev.get('is_charging') or curr.get('is_charging'):
                        continue
                    
                    config = curr.get('config')
                    if not config:
                        continue
                    
                    dt = curr['timestamp'] - prev['timestamp']
                    if dt > 0 and dt < 10:  # Skip large gaps
                        battery_change = prev['battery'] - curr['battery']
                        if battery_change > 0:  # Only count actual drain
                            drain_rate = battery_change / dt  # % per second
                            
                            if config not in drain_by_config:
                                drain_by_config[config] = []
                            drain_by_config[config].append(drain_rate)
            
            if drain_by_config:
                avg_drain_rates = {}
                for config, rates in drain_by_config.items():
                    avg_drain_rates[config] = {
                        'avg_rate_per_second': sum(rates) / len(rates),
                        'avg_rate_per_minute': (sum(rates) / len(rates)) * 60,
                        'min_rate': min(rates),
                        'max_rate': max(rates)
                    }
                stats['battery_drain_rates_by_config'] = avg_drain_rates
        
        return stats


def main():
    rclpy.init()
    monitor = RealSimMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if monitor.mission_started:
            monitor.finalize_run()
        
        if monitor.runs:
            monitor.save_results()
            
            stats = monitor.calculate_statistics()
            monitor.get_logger().info(f"\n{'='*60}")
            monitor.get_logger().info("FINAL SUMMARY")
            monitor.get_logger().info(f"{'='*60}")
            monitor.get_logger().info(f"Runs: {stats['total_runs']}")
            monitor.get_logger().info(f"Success: {stats['successful_runs']} ({stats['success_rate']:.1f}%)")
            monitor.get_logger().info(f"Avg Duration: {stats['avg_duration']:.1f}s")
            monitor.get_logger().info(f"Avg Distance: {stats['avg_distance']:.2f}m")
            monitor.get_logger().info(f"Total Distance: {stats['total_distance']:.2f}m")
            
            if stats.get('avg_controller_params_by_config'):
                monitor.get_logger().info(f"\n📊 Average Controller Parameters by Config:")
                for config, params in stats['avg_controller_params_by_config'].items():
                    monitor.get_logger().info(f"\n  {config}:")
                    for param, value in sorted(params.items()):
                        short_name = param.replace('FollowPath.', '')
                        monitor.get_logger().info(f"    {short_name:25s} = {value:.3f}")
                
                # Show ranges if available
                if stats.get('param_ranges_by_config'):
                    monitor.get_logger().info(f"\n📊 Parameter Ranges by Config:")
                    for config, param_ranges in stats['param_ranges_by_config'].items():
                        monitor.get_logger().info(f"\n  {config}:")
                        for param, ranges in sorted(param_ranges.items()):
                            short_name = param.replace('FollowPath.', '')
                            monitor.get_logger().info(
                                f"    {short_name:25s}: "
                                f"min={ranges['min']:.3f}, "
                                f"avg={ranges['avg']:.3f}, "
                                f"max={ranges['max']:.3f}")
            
            if stats.get('battery_drain_rates_by_config'):
                monitor.get_logger().info(f"\n🔋 Battery Drain Rates by Config:")
                for config, rates in stats['battery_drain_rates_by_config'].items():
                    monitor.get_logger().info(f"\n  {config}:")
                    monitor.get_logger().info(f"    Avg: {rates['avg_rate_per_minute']:.2f}% per minute")
                    monitor.get_logger().info(f"    Range: {rates['min_rate']*60:.2f}% - {rates['max_rate']*60:.2f}% per minute")
            
            monitor.get_logger().info(f"{'='*60}\n")
        
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()