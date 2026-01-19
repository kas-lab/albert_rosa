#!/usr/bin/env python3
"""
Improved Benchmark Monitor - Tracks moves from action events
=============================================================
Uses /action_events topic published by move_action.cpp
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
import json
import time
from datetime import datetime
from pathlib import Path
from std_msgs.msg import Bool



class BenchmarkMonitor(Node):
    def __init__(self):
        super().__init__('benchmark_monitor')
        
        # Subscribe to run completion
        self.run_complete_sub = self.create_subscription(
            Int32,
            '/benchmark/run_complete',
            self.run_complete_callback,
            10
        )
        
        # Subscribe to battery
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        self.recharge_sub = self.create_subscription(
            Bool,
            '/battery_monitor/recharge_complete',
            self.recharge_complete_callback,
            10
        )
        # Subscribe to diagnostics for config changes
        # self.diagnostics_sub = self.create_subscription(
        #     DiagnosticArray,
        #     '/diagnostics',
        #     self.diagnostics_callback,
        #     10
        # )
        
        # Subscribe to action events (THE SAME SOURCE AS LOGS!)
        self.action_events_sub = self.create_subscription(
            String,
            '/action_events',
            self.action_event_callback,
            10
        )
        self.monitoring_sub = self.create_subscription(
            DiagnosticArray,
            '/navigation/monitoring',  # ← New topic!
            self.monitoring_callback,
            10
        )
        
        # Run tracking
        self.runs = []
        self.current_run = {
            'run_number': 0,
            'start_time': None,
            'end_time': None,
            'start_battery': None,
            'end_battery': None,
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
        }
        
        self.last_config = None
        self.config_start_time = time.time()

        self.last_perception = None
        self.last_arm_level = None
        
        # Results directory
        self.results_dir = Path('benchmark_results')
        self.results_dir.mkdir(exist_ok=True)
        
        self.get_logger().info('🔬 Benchmark Monitor ready!')
        self.get_logger().info('   Tracking via /action_events')
    
    def battery_callback(self, msg):
        """Track battery changes"""
        if msg.percentage is None:
            return
        
        battery = msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage
        
        if not (0 <= battery <= 100):
            return
        
        if self.current_run['start_battery'] is None:
            self.current_run['start_battery'] = battery
        
        self.current_run['end_battery'] = battery
        
        if battery < 1.0 and not self.current_run.get('failed', False):
            self.current_run['success'] = False
            self.current_run['failed'] = True
            self.get_logger().error(
                f'❌ Run {self.current_run["run_number"]}: Battery depleted!')
            
    def recharge_complete_callback(self, msg: Bool):
        if msg.data:
            self.current_run['recharge_count'] += 1
            self.get_logger().info(
                f"  ⚡ Recharge #{self.current_run['recharge_count']} (BatteryMonitor)"
            )

    
    def monitoring_callback(self, msg: DiagnosticArray):
        """Track ROSA config changes and adaptations"""
        current_time = time.time()
        
        for status in msg.status:
            if status.name == 'navigation_monitoring':
                for kv in status.values:
                    # Track config changes
                    if kv.key == 'current-configuration':
                        config = kv.value
                        
                        # Track usage time
                        if self.last_config is not None:
                            duration = current_time - self.config_start_time
                            if self.last_config not in self.current_run['config_usage']:
                                self.current_run['config_usage'][self.last_config] = 0.0
                            self.current_run['config_usage'][self.last_config] += duration
                        
                        # Track config change (adaptation)
                        if not self.current_run['config_changes'] or \
                           self.current_run['config_changes'][-1] != config:
                            self.current_run['config_changes'].append(config)
                            self.current_run['adaptations'] += 1
                            
                            # Read reason directly from C++
                            reason = "unknown"
                            for kv2 in status.values:
                                if kv2.key == 'adaptation-reason':
                                    reason = kv2.value
                                    break
                            
                            event = f"{config} (reason: {reason})"
                            self.current_run['adaptation_events'].append(event)
                            
                            self.get_logger().info(
                                f"  🔄 Adaptation {self.current_run['adaptations']}: "
                                f"{config} (reason: {reason})")
                        
                        self.last_config = config
                        self.config_start_time = current_time
                    elif kv.key == 'perception-active':
                        perception = kv.value.lower() == 'true'
                        
                        if self.last_perception is not None and perception != self.last_perception:
                            self.current_run['perception_changes'] += 1
                            state = 'ON' if perception else 'OFF'
                            event = f"Perception → {state}"
                            self.current_run['component_events'].append(event)
                            
                            self.get_logger().info(
                                f"  👁️ Component adaptation: Perception → {state}")
                        
                        self.last_perception = perception
                    
                    # ✅ ADD: Track arm power level changes
                    elif kv.key == 'arm-power-level':
                        arm_level = kv.value
                        
                        if self.last_arm_level is not None and arm_level != self.last_arm_level:
                            self.current_run['arm_changes'] += 1
                            event = f"Arm power → {arm_level.upper()}"
                            self.current_run['component_events'].append(event)
                            
                            self.get_logger().info(
                                f"  🦾 Component adaptation: Arm → {arm_level.upper()}")
                        
                        self.last_arm_level = arm_level
    
    def action_event_callback(self, msg):
        """Track moves from action events - SAME SOURCE AS LOGS!"""
        parts = msg.data.split('|')
        event_type = parts[0]
        
        if event_type == "MOVE_START":
            from_wp = parts[1]
            to_wp = parts[2]
            config = parts[3]
            distance = float(parts[4]) if len(parts) > 4 else 5.0
            action_type = parts[5] if len(parts) > 5 else "move_unknown"
            
            self.current_run['move_actions'] += 1
            self.current_run['distance_traveled'] += distance
            
            move_detail = {
                'number': self.current_run['move_actions'],
                'from': from_wp,
                'to': to_wp,
                'config': config,
                'distance': distance,
                'action_type': action_type 

            }
            self.current_run['move_details'].append(move_detail)
            
            self.get_logger().info(
                f"  🚗 Move #{self.current_run['move_actions']}: "
                f"{from_wp} → {to_wp} ({config}, {action_type}) [{distance:.2f}m]")
        
        elif event_type == "MOVE_END":
            from_wp = parts[1]
            to_wp = parts[2]
            self.get_logger().debug(f"  ✅ Move completed: {from_wp} → {to_wp}")
        
        elif event_type == "RECHARGE_START":
            wp = parts[1] if len(parts) > 1 else "unknown"
            self.current_run['recharge_count'] += 1
            self.get_logger().info(
                f"  ⚡ Recharge #{self.current_run['recharge_count']} at {wp}")
        
        elif event_type == "RECHARGE_END":
            self.get_logger().debug("  ✅ Recharge completed")
    
    def run_complete_callback(self, msg):
        """Called when navigate.cpp completes a run"""
        run_num = msg.data
        
        # Finalize previous run
        if self.current_run['run_number'] > 0:
            self.finalize_run()
            self.save_results()
        
        # Start new run
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
        }
        
        self.last_config = None
        self.config_start_time = time.time()
        self.last_perception = None
        self.last_arm_level = None
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🔬 Monitoring Run {run_num}...")
        self.get_logger().info(f"{'='*60}\n")
    
    def finalize_run(self):
        """Finalize current run data"""
        self.current_run['end_time'] = datetime.now().isoformat()
        
        # Calculate duration
        start = datetime.fromisoformat(self.current_run['start_time'])
        end = datetime.fromisoformat(self.current_run['end_time'])
        duration = (end - start).total_seconds()
        self.current_run['duration_seconds'] = duration
        
        # Finalize config usage
        if self.last_config is not None:
            final_duration = time.time() - self.config_start_time
            if self.last_config not in self.current_run['config_usage']:
                self.current_run['config_usage'][self.last_config] = 0.0
            self.current_run['config_usage'][self.last_config] += final_duration
        
        # Calculate battery used
        if self.current_run['start_battery'] is not None and \
           self.current_run['end_battery'] is not None:
            battery_used = self.current_run['start_battery'] - self.current_run['end_battery']
        else:
            battery_used = None
        
        self.current_run['battery_used'] = battery_used
        
        # Save run
        self.runs.append(self.current_run.copy())
        
        # Print detailed run summary
        self.print_run_summary()
    
    def print_run_summary(self):
        """Print comprehensive run summary"""
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
        
        self.get_logger().info(f"🚗 Move actions: {run['move_actions']}")
        self.get_logger().info(f"⚡ Recharges: {run['recharge_count']}")
        self.get_logger().info(f"📏 Distance: ~{run['distance_traveled']:.1f}m")
        
        self.get_logger().info(f"🔄 Adaptations: {run['adaptations']}")
        if run['adaptation_events']:
            for i, event in enumerate(run['adaptation_events'], 1):
                self.get_logger().info(f"   {i}. {event}")

        total_component = run.get('perception_changes', 0) + run.get('arm_changes', 0)
        if total_component > 0:
            self.get_logger().info(
                f"🔧 Component adaptations: {total_component} "
                f"(👁️ {run.get('perception_changes', 0)} perception, "
                f"🦾 {run.get('arm_changes', 0)} arm)")
            
            if run.get('component_events'):
                for event in run['component_events']:
                    self.get_logger().info(f"   • {event}")
        
        self.get_logger().info("⚙️  Configuration usage:")
        for config, duration in run['config_usage'].items():
            self.get_logger().info(f"   {config}: {duration:.1f}s")
        
        self.get_logger().info(f"{'='*60}\n")
    
    def save_results(self):
        """Save all runs to JSON"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = self.results_dir / f'benchmark_{timestamp}.json'
        
        summary = {
            'total_runs': len(self.runs),
            'timestamp': datetime.now().isoformat(),
            'runs': self.runs,
            'statistics': self.calculate_statistics()
        }
        
        with open(filepath, 'w') as f:
            json.dump(summary, f, indent=2)
        
        self.get_logger().info(f"💾 Saved to {filepath}")
    
    def calculate_statistics(self):
        """Calculate aggregate statistics"""
        if not self.runs:
            return {}
        
        successful = [r for r in self.runs if r.get('success', True)]
        
        stats = {
            'total_runs': len(self.runs),
            'successful_runs': len(successful),
            'success_rate': len(successful) / len(self.runs) * 100,
            'avg_duration': sum(r.get('duration_seconds', 0) for r in self.runs) / len(self.runs),
            'avg_adaptations': sum(r['adaptations'] for r in self.runs) / len(self.runs),
            'total_adaptations': sum(r['adaptations'] for r in self.runs),
            'avg_move_actions': sum(r['move_actions'] for r in self.runs) / len(self.runs),
            'avg_recharges': sum(r['recharge_count'] for r in self.runs) / len(self.runs),
            'avg_distance': sum(r['distance_traveled'] for r in self.runs) / len(self.runs),
            'avg_perception_changes': sum(r.get('perception_changes', 0) for r in self.runs) / len(self.runs),
            'avg_arm_changes': sum(r.get('arm_changes', 0) for r in self.runs) / len(self.runs),
            'total_component_adaptations': sum(r.get('perception_changes', 0) + r.get('arm_changes', 0) for r in self.runs),
        }
        
        # Battery stats
        battery_values = [r['battery_used'] for r in self.runs if r.get('battery_used')]
        if battery_values:
            stats['avg_battery_used'] = sum(battery_values) / len(battery_values)
        
        # Config usage
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
        
        return stats


def main():
    rclpy.init()
    monitor = BenchmarkMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if monitor.runs:
            monitor.save_results()
            
            stats = monitor.calculate_statistics()
            monitor.get_logger().info(f"\n{'='*60}")
            monitor.get_logger().info("FINAL BENCHMARK SUMMARY")
            monitor.get_logger().info(f"{'='*60}")
            monitor.get_logger().info(f"Total Runs: {stats['total_runs']}")
            monitor.get_logger().info(
                f"Successful: {stats['successful_runs']} ({stats['success_rate']:.1f}%)")
            monitor.get_logger().info(f"Avg Duration: {stats['avg_duration']:.1f}s")
            monitor.get_logger().info(f"Avg Adaptations: {stats['avg_adaptations']:.2f}")
            monitor.get_logger().info(f"Avg Moves: {stats['avg_move_actions']:.1f}")
            monitor.get_logger().info(f"Avg Recharges: {stats['avg_recharges']:.2f}")
            monitor.get_logger().info(f"Avg Distance: {stats['avg_distance']:.1f}m")
            
            if stats.get('avg_battery_used'):
                monitor.get_logger().info(
                    f"Avg Battery: {stats['avg_battery_used']:.1f}%")
            
            monitor.get_logger().info("\nConfig Usage:")
            for config, pct in stats.get('config_usage_percentage', {}).items():
                monitor.get_logger().info(f"  {config}: {pct:.1f}%")
            
            monitor.get_logger().info(f"{'='*60}\n")
        
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()