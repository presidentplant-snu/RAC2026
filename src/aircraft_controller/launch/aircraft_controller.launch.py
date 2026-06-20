"""Bring up the aircraft controller stack.

Launches:
  - MicroXRCEAgent (udp4 -p 8888)
  - translation_node
  - vision_tracker
  - aircraft_controller (mission_runner)
  - aircraft_logger        (only when logging:=true)

Set the `logging` launch argument to also bring up the logger:
    ros2 launch aircraft_controller aircraft_controller.launch.py logging:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    logging = LaunchConfiguration('logging')

    declare_logging = DeclareLaunchArgument(
        'logging',
        default_value='false',
        description='Also launch aircraft_logger when true.',
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    translation_node = Node(
        package='translation_node',
        executable='translation_node_bin',
        name='translation_node',
        output='screen',
    )

    vision_tracker = Node(
        package='vision_tracker',
        executable='vision_tracker',
        name='vision_tracker',
        output='screen',
    )

    aircraft_controller = Node(
        package='aircraft_controller',
        executable='mission_runner',
        name='mission_runner',
        output='screen',
    )

    aircraft_logger = Node(
        package='aircraft_logger',
        executable='aircraft_logger',
        name='aircraft_logger',
        output='screen',
        condition=IfCondition(logging),
    )

    return LaunchDescription([
        declare_logging,
        micro_xrce_agent,
        translation_node,
        vision_tracker,
        aircraft_controller,
        aircraft_logger,
    ])
