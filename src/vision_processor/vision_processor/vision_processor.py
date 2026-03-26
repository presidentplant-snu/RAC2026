import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from aircraft_msgs.msg import AircraftState, CameraTrackedObject

import threading
import time
import numpy as np
import cv2 as cv

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from .detectors import ArucoDetector
from .pipeline import VideoPipelineBase, SIYIPipeline, GazeboPipeline

class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')
        Gst.init(None)

        self.aircraft_state: AircraftState = None
        self.tracked_object: CameraTrackedObject = None 

        self.frame: np.ndarray | None = None
        self.pipeline: VideoPipelineBase
        
        self.setup_ros2()

        if self.use_gazebo:
            self.pipeline = GazeboPipeline()
        else:
            self.pipeline = SIYIPipeline(target_ip="srt://192.168.10.36:5000")
        
        self.pipeline_thread = threading.Thread(target=self.pipeline.run, daemon=True)
        self.pipeline_thread.start()

        self.camera_w = 1920
        self.camera_h = 1920
        self.camera_fov_w = 80 # degrees
        self.camera_fov_h = 80*9/16 # degrees

        self.aruco_detector = ArucoDetector()

    def setup_ros2(self):
        self.aircraft_state_subscriber = self.create_subscription(
                AircraftState, '/aircraft_state',
                self._aircraft_state_callback,
                QoSPresetProfiles.SERVICES_DEFAULT.value
                )

        self.tracked_object_publisher = self.create_publisher(
                CameraTrackedObject, '/tracked_object',
                QoSPresetProfiles.SERVICES_DEFAULT.value
                )

        # Set to 10 Hz, might change later depending on detection speed.
        self.main_timer = self.create_timer(0.10, self._main_timer_callback)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_gazebo', False),
                ('show_debug', False)
            ]
        )

        self.use_gazebo = self.get_parameter('use_gazebo').value
        self.use_gazebo = True
        self.show_debug = self.get_parameter('show_debug').value
        self.show_debug = True

    def _aircraft_state_callback(self, msg):
        self.aircraft_state = msg

    def _main_timer_callback(self):
        self.aircraft_state = AircraftState
        self.aircraft_state.track_type = AircraftState.TRACK_ARUCO
        if self.aircraft_state is None:
            return

        self.frame = self.pipeline.get_frame() 
        if self.frame is None:
            return

        match self.aircraft_state.track_type:
            case AircraftState.TRACK_PATIENT:
                pass
            case AircraftState.TRACK_ARUCO:
                detection = self.aruco_detector.detect(self.frame)
                if detection is not None:
                    self.tracked_object  = CameraTrackedObject()
                    cv.circle(self.frame, (int(detection[0]), int(detection[1])), 5, (0, 255, 0), -1)
                    self.tracked_object.tan_x, self.tracked_object.tan_y = \
                            self.pixel_to_angle(detection[0], detection[1])
            case _:
                pass

        if self.show_debug:
            cv.imshow("Debug", self.frame)
            cv.waitKey(1)

        if self.tracked_object is None:
            return

        self.tracked_object_publisher.publish(self.tracked_object)




    def pixel_to_angle(self, x, y):
        # Up / Right is +ve

        angle_x = np.tan(np.deg2rad((x-self.camera_w/2) * (self.camera_fov_w / self.camera_w)))
        angle_y = -np.tan(np.deg2rad((y-self.camera_h/2) * (self.camera_fov_h / self.camera_h)))
        
        return angle_x, angle_y

def main(args=None):
    rclpy.init(args=args)

    visionProcessorNode = VisionProcessorNode()

    rclpy.spin(visionProcessorNode)

    visionProcessorNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

