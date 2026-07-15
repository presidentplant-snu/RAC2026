"""Bring up the vision processor.

Reads the camera stream, runs detection (ArUco / patient ONNX) and publishes
tracked objects on /camera_tracked_object.

All parameters (camera geometry, pipeline selection, model path, SIYI restream
URI) come from config/vision.yaml. Edit that file to reconfigure.

Example:
    ros2 launch aircraft_bringup vision_processor.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aircraft_bringup')
    vision_config = os.path.join(pkg_share, 'config', 'vision.yaml')

    vision_processor = Node(
        package='vision_processor',
        executable='vision_processor',
        name='vision_processor',
        output='screen',
        parameters=[vision_config],
    )

    return LaunchDescription([
        vision_processor,
    ])
