"""Bring up the SRT stream viewer.

Displays the SRT video stream and overlays tracked-object markers, delayed by
the SRT latency so the markers line up with the buffered video.

Launch arguments:
    srt_uri      SRT URI to listen on (default: srt://:5000)
    srt_latency  srtsrc receive-buffer latency in ms; also used to delay the
                 marker overlay to match the video (default: 100)

Example:
    ros2 launch aircraft_bringup stream_viewer.launch.py srt_latency:=300
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    srt_uri = LaunchConfiguration('srt_uri')
    srt_latency = LaunchConfiguration('srt_latency')

    declare_srt_uri = DeclareLaunchArgument(
        'srt_uri',
        default_value='srt://:5000',
        description='SRT URI to listen on.',
    )

    declare_srt_latency = DeclareLaunchArgument(
        'srt_latency',
        default_value='100',
        description='srtsrc latency in ms, also used to delay the overlay.',
    )

    stream_viewer = Node(
        package='vision_processor',
        executable='stream_viewer',
        name='stream_viewer',
        output='screen',
        parameters=[{
            'srt_uri': srt_uri,
            'srt_latency': ParameterValue(srt_latency, value_type=int),
        }],
    )

    return LaunchDescription([
        declare_srt_uri,
        declare_srt_latency,
        stream_viewer,
    ])
