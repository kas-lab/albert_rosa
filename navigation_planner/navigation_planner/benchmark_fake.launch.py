# Copyright 2025
#
# Launch ROSA stack in FAKE EXECUTION mode for benchmarking
# - TypeDB running
# - ROSA reasoning active
# - PlanSys2 planning
# - navigate.cpp orchestrating (fake_execution=True)
# - NO Gazebo, NO Nav2, NO real action nodes

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_task_plan_path = get_package_share_directory('navigation_task_plan')
    plansys_path = get_package_share_directory('plansys2_bringup')
    
    # Launch arguments
    fake_execution_arg = DeclareLaunchArgument(
        'fake_execution',
        default_value='true',
        description='Enable fake execution mode (no real actions)'
    )
    
    enable_proactive_arg = DeclareLaunchArgument(
        'enable_proactive',
        default_value='true',
        description='Enable proactive reasoning in navigate.cpp'
    )
    
    # PlanSys2 (task planning)
    plansys2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            plansys_path,
            'launch',
            'plansys2_bringup_launch_distributed.py'
        )),
        launch_arguments={
            'model_file': navigation_task_plan_path + '/pddl/domain.pddl',
        }.items()
    )
    
    # Navigate controller (orchestrator)
    navigation_controller_node = Node(
        package='navigation_task_plan',
        executable='navigate',
        parameters=[
            {'rosa_actions': ['move_dark', 'move_lit', 'recharge', 'move_to_recharge']},
            {'enable_proactive': LaunchConfiguration('enable_proactive')},
            {'fake_execution': LaunchConfiguration('fake_execution')}
        ],
        output='screen'
    )
    
    # Fake action executors (just for PlanSys2 interface)
    # These complete instantly when fake_execution=True
    fake_move_lit_node = Node(
        package='navigation_task_plan',
        executable='fake_action_executor',
        name='action_move_lit',
        parameters=[
            {'action_name': 'move_lit'},
            {'fake_execution': LaunchConfiguration('fake_execution')},
            {'execution_time': 0.1}  # Instant completion
        ]
    )
    
    fake_move_dark_node = Node(
        package='navigation_task_plan',
        executable='fake_action_executor',
        name='action_move_dark',
        parameters=[
            {'action_name': 'move_dark'},
            {'fake_execution': LaunchConfiguration('fake_execution')},
            {'execution_time': 0.1}
        ]
    )
    
    fake_recharge_node = Node(
        package='navigation_task_plan',
        executable='fake_action_executor',
        name='action_recharge',
        parameters=[
            {'action_name': 'recharge'},
            {'fake_execution': LaunchConfiguration('fake_execution')},
            {'execution_time': 0.1}
        ]
    )
    
    fake_move_to_recharge_node = Node(
        package='navigation_task_plan',
        executable='fake_action_executor',
        name='action_move_to_recharge',
        parameters=[
            {'action_name': 'move_to_recharge'},
            {'fake_execution': LaunchConfiguration('fake_execution')},
            {'execution_time': 0.1}
        ]
    )
    
    return LaunchDescription([
        fake_execution_arg,
        enable_proactive_arg,
        plansys2_bringup,
        navigation_controller_node,
        fake_move_lit_node,
        fake_move_dark_node,
        fake_recharge_node,
        fake_move_to_recharge_node,
    ])