"""Bring up the SRT stream viewer.

Displays the SRT video stream and overlays tracked-object markers, delayed by
the SRT latency so the markers line up with the buffered video.

All parameters (SRT URI/latency, marker timeout, camera FOV) come from
config/vision.yaml, the same file the vision_processor loads. Edit that file to
reconfigure.

Example:
    ros2 launch aircraft_bringup stream_viewer.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aircraft_bringup')
    vision_config = os.path.join(pkg_share, 'config', 'vision.yaml')

    stream_viewer = Node(
        package='vision_processor',
        executable='stream_viewer',
        name='stream_viewer',
        output='screen',
        parameters=[vision_config],
    )

    return LaunchDescription([
        stream_viewer,
    ])
