"""Bring up the vision processor.

Reads the camera stream, runs detection (ArUco / patient ONNX) and publishes
tracked objects on /camera_tracked_object.

Launch arguments:
    use_gazebo          use the Gazebo pipeline instead of the SIYI camera
                        (default: false)
    show_debug          show the annotated debug window (default: false)
    patient_model_path  path to the patient ONNX model
                        (default: /home/radxa/RAC2026/model/yolo26n_1280_3out_int8.onnx)

Example:
    ros2 launch aircraft_bringup vision_processor.launch.py use_gazebo:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_gazebo = LaunchConfiguration('use_gazebo')
    show_debug = LaunchConfiguration('show_debug')
    patient_model_path = LaunchConfiguration('patient_model_path')

    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Use the Gazebo pipeline instead of the SIYI camera.',
    )

    declare_show_debug = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='Show the annotated debug window.',
    )

    declare_patient_model_path = DeclareLaunchArgument(
        'patient_model_path',
        default_value='/home/radxa/RAC2026/model/yolo26n_1280_3out_int8.onnx',
        description='Path to the patient ONNX model (empty disables it).',
    )

    vision_processor = Node(
        package='vision_processor',
        executable='vision_processor',
        name='vision_processor',
        output='screen',
        parameters=[{
            'use_gazebo': ParameterValue(use_gazebo, value_type=bool),
            'show_debug': ParameterValue(show_debug, value_type=bool),
            'patient_model_path': patient_model_path,
        }],
    )

    return LaunchDescription([
        declare_use_gazebo,
        declare_show_debug,
        declare_patient_model_path,
        vision_processor,
    ])
