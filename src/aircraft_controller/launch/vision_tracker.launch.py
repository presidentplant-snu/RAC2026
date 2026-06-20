"""Bring up the vision tracker stack.

Launches:
  - MicroXRCEAgent (udp4 -p 8888)
  - translation_node
  - vision_tracker
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([
        micro_xrce_agent,
        translation_node,
        vision_tracker,
    ])
