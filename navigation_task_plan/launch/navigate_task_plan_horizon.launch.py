import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    navigation_task_plan_path = get_package_share_directory('navigation_task_plan')
    plansys_path = get_package_share_directory('plansys2_bringup')

    benchmark_mode_arg = DeclareLaunchArgument(
        'benchmark_mode',
        default_value='false',
        description='Enable benchmark auto-restart mode'
    )
    
    max_runs_arg = DeclareLaunchArgument(
        'max_benchmark_runs',
        default_value='100',
        description='Number of benchmark runs to execute'
    )
    
    debug_arg = DeclareLaunchArgument(
        'enable_debug_in_benchmark',
        default_value='false',
        description='Show DEBUG/INFO logs during benchmark'
    )
    
    # ✅ ADD THIS: Scenario difficulty argument
    difficulty_arg = DeclareLaunchArgument(
        'scenario_difficulty',
        default_value='easy',
        description='Map difficulty: easy, medium, or hard'
    )
    
    plansys2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            plansys_path, 'launch', 'plansys2_bringup_launch_distributed.py'
        )),
        launch_arguments={
            'model_file': navigation_task_plan_path + '/pddl/domain_sas.pddl',
            'problem_file': navigation_task_plan_path + '/pddl/problem_sas.pddl',
        }.items()
    )
    
    navigation_controller_node = Node(
        package='navigation_task_plan',
        executable='navigate_horizon',
        parameters=[
            {'rosa_actions': ['move_dark', 'move_lit', 'recharge', 'move_to_recharge']},
            {'benchmark_mode': LaunchConfiguration('benchmark_mode')},
            {'max_benchmark_runs': LaunchConfiguration('max_benchmark_runs')},
            {'enable_debug_in_benchmark': LaunchConfiguration('enable_debug_in_benchmark')},
            {'scenario_difficulty': LaunchConfiguration('scenario_difficulty')},  # ✅ ADD THIS
        ],
        output='screen'
    )

    pddl_move_action_node_dark = Node(
        package='navigation_task_plan',
        executable='action_move',
        name='action_move_dark',
        parameters=[
            os.path.join(navigation_task_plan_path, 'config', 'waypoints_dark.yaml'),
            {'action_name': 'move_dark'},
            {'fake_execution': False}
        ]
    )

    pddl_move_action_node_lit = Node(
        package='navigation_task_plan',
        executable='action_move',
        name='action_move_lit',
        parameters=[
            os.path.join(navigation_task_plan_path, 'config', 'waypoints_lit.yaml'),
            {'action_name': 'move_lit'},
            {'fake_execution': False}
        ]
    )

    move_to_recharge_node = Node(
        package='navigation_task_plan',
        executable='action_move',
        name='action_move_to_recharge',
        parameters=[
            os.path.join(navigation_task_plan_path, 'config', 'waypoints_to_recharge.yaml'),
            {'action_name': 'move_to_recharge'},
            {'fake_execution': False}
        ]
    )

    action_recharge_node = Node(
        package='navigation_task_plan',
        executable='action_recharge',
        name='action_recharge',
        output='screen',
        parameters=[{'action_name': 'recharge'}]
    )

    return LaunchDescription([
        benchmark_mode_arg,
        max_runs_arg,
        debug_arg,
        difficulty_arg,  # ✅ ADD THIS
        plansys2_bringup,
        navigation_controller_node,
        pddl_move_action_node_dark,
        pddl_move_action_node_lit,
        action_recharge_node,
        move_to_recharge_node,
    ])