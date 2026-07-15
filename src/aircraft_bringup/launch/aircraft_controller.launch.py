"""Bring up the aircraft controller stack.

Requires MicroXRCEAgent to be running separately (e.g. `MicroXRCEAgent udp4
-p 8888`); it is intentionally not launched here.

Launches:
  - translation_node
  - vision_tracker         (tuning params from config/vision_tracker.yaml)
  - aircraft_controller (mission_runner)
  - aircraft_logger        (only when logging:=true)

Launch arguments:
    mission   mission JSON file to run, resolved against this package's
              missions/ directory (default: vtol_fw_mission.json)
    logging   also bring up aircraft_logger when true (default: false)

Example:
    ros2 launch aircraft_bringup aircraft_controller.launch.py \
        mission:=precland.json logging:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aircraft_bringup')
    vision_config = os.path.join(pkg_share, 'config', 'vision_tracker.yaml')

    logging = LaunchConfiguration('logging')
    mission = LaunchConfiguration('mission')

    declare_logging = DeclareLaunchArgument(
        'logging',
        default_value='false',
        description='Also launch aircraft_logger when true.',
    )

    declare_mission = DeclareLaunchArgument(
        'mission',
        default_value='vtol_fw_mission.json',
        description='Mission JSON file, resolved against the package missions/ dir.',
    )

    mission_file = PathJoinSubstitution([pkg_share, 'missions', mission])

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
        parameters=[vision_config],
    )

    aircraft_controller = Node(
        package='aircraft_controller',
        executable='mission_runner',
        name='mission_runner',
        output='screen',
        parameters=[{'mission_file': mission_file}],
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
        declare_mission,
        translation_node,
        vision_tracker,
        aircraft_controller,
        aircraft_logger,
    ])
