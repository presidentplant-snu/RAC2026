"""Helper node that publishes an AircraftState with just `track_type` set.

Lets you manually drive the vision_processor's tracking mode without the
full aircraft_controller running.

Usage:
    ros2 run vision_processor set_track_type aruco
    ros2 run vision_processor set_track_type patient
    ros2 run vision_processor set_track_type none

    # or via parameter
    ros2 run vision_processor set_track_type --ros-args -p track_type:=aruco
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from aircraft_msgs.msg import AircraftState

# Accepts short ('aruco') and full ('track_aruco') spellings.
TRACK_TYPES = {
    'none': AircraftState.TRACK_NONE,
    'track_none': AircraftState.TRACK_NONE,
    'patient': AircraftState.TRACK_PATIENT,
    'track_patient': AircraftState.TRACK_PATIENT,
    'aruco': AircraftState.TRACK_ARUCO,
    'track_aruco': AircraftState.TRACK_ARUCO,
}


class SetTrackTypeNode(Node):
    def __init__(self, default_track_type='aruco'):
        super().__init__('set_track_type')

        self.declare_parameter('track_type', default_track_type)
        name = str(self.get_parameter('track_type').value).strip().lower()

        if name not in TRACK_TYPES:
            valid = ', '.join(sorted(set(TRACK_TYPES)))
            raise ValueError(
                f"Unknown track_type '{name}'. Valid options: {valid}")

        self.track_type = TRACK_TYPES[name]
        self.track_name = name

        self.publisher = self.create_publisher(
            AircraftState, '/aircraft_state',
            QoSPresetProfiles.SERVICES_DEFAULT.value)

        # Republish periodically so late subscribers still pick it up.
        self.timer = self.create_timer(0.5, self._publish)
        self.get_logger().info(
            f"Publishing aircraft_state with track_type={self.track_name} "
            f"({self.track_type}) on /aircraft_state")

    def _publish(self):
        msg = AircraftState()
        msg.track_type = self.track_type
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Allow a bare positional argument, e.g. `ros2 run ... set_track_type aruco`.
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    default_track_type = argv[0] if argv else 'aruco'

    node = SetTrackTypeNode(default_track_type=default_track_type)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
