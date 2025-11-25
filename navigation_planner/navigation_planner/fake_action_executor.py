#!/usr/bin/env python3
"""
Fake Action Executor
====================
Replaces action_move, action_recharge nodes for benchmarking.
Reports success instantly when fake_execution=True.

This allows navigate.cpp + ROSA + PlanSys2 to run the FULL
adaptation loop without needing Gazebo/Nav2!
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from plansys2_executor.ActionExecutorClient import ActionExecutorClient
from lifecycle_msgs.msg import Transition
import time


class FakeActionExecutor(ActionExecutorClient):
    """
    Fake action executor that completes instantly
    Allows benchmarking without real robot execution
    """
    
    def __init__(self, node_name: str = 'fake_action'):
        super().__init__(node_name, 0.5)  # 500ms rate
        
        # Parameters
        self.declare_parameter('fake_execution', True)
        self.declare_parameter('execution_time', 0.1)  # Instant
        self.declare_parameter('action_name', 'move_lit')
        
        self.fake_execution = self.get_parameter('fake_execution').value
        self.execution_time = self.get_parameter('execution_time').value
        self.action_name = self.get_parameter('action_name').value
        
        self.get_logger().info(f"🎭 Fake executor: {self.action_name}")
        self.get_logger().info(f"   fake_execution={self.fake_execution}")
        self.get_logger().info(f"   execution_time={self.execution_time}s")
        
        self.start_time = None
    
    def on_activate(self, state):
        """Called when action starts"""
        self.get_logger().info(f"▶️  Starting {self.action_name}")
        self.start_time = time.time()
        return super().on_activate(state)
    
    def do_work(self):
        """Called periodically during execution"""
        if self.start_time is None:
            return
        
        elapsed = time.time() - self.start_time
        
        if elapsed >= self.execution_time:
            # Action complete!
            self.get_logger().info(f"✅ {self.action_name} completed (fake)")
            self.finish(True, 1.0, f"{self.action_name} complete")
            self.start_time = None
        else:
            # Progress feedback
            progress = elapsed / self.execution_time
            self.send_feedback(progress, f"{self.action_name} running")


def main(args=None):
    rclpy.init(args=args)
    
    node = FakeActionExecutor()
    node.trigger_transition(Transition.TRANSITION_CONFIGURE)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node.get_node_base_interface())
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()