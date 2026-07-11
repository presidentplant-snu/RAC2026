import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from aircraft_msgs.msg import CameraTrackedObject

import threading
import time
import numpy as np
import cv2 as cv

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from .pipeline import SRTPipeline

class StreamViewerNode(Node):
    def __init__(self):
        super().__init__('stream_viewer_node')
        Gst.init(None)

        self.tracked_object: CameraTrackedObject | None = None
        self.tracked_object_time: float = 0.0

        # Must match the camera parameters in vision_processor.py
        self.camera_fov_w = 80 # degrees
        self.camera_fov_h = 80*9/16 # degrees

        self.setup_ros2()

        self.pipeline = SRTPipeline(listen_uri=self.srt_uri)
        self.pipeline_thread = threading.Thread(target=self.pipeline.run, daemon=True)
        self.pipeline_thread.start()

        self.get_logger().info(f"Listening for SRT stream on {self.srt_uri}")

    def setup_ros2(self):
        self.tracked_object_subscriber = self.create_subscription(
                CameraTrackedObject, '/camera_tracked_object',
                self._tracked_object_callback,
                QoSPresetProfiles.SERVICES_DEFAULT.value
                )

        # ~30 fps display
        self.display_timer = self.create_timer(1/30, self._display_timer_callback)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('srt_uri', 'srt://:5000'),
                ('marker_timeout', 1.0)
            ]
        )

        self.srt_uri = self.get_parameter('srt_uri').value
        self.marker_timeout = self.get_parameter('marker_timeout').value

    def _tracked_object_callback(self, msg):
        self.tracked_object = msg
        self.tracked_object_time = time.monotonic()

    def _display_timer_callback(self):
        frame = self.pipeline.get_frame()
        if frame is None:
            return

        frame = frame.copy()

        if self.tracked_object is not None and \
                time.monotonic() - self.tracked_object_time < self.marker_timeout:
            h, w = frame.shape[:2]
            x, y = self.angle_to_pixel(
                    self.tracked_object.tan_x, self.tracked_object.tan_y, w, h)

            cv.circle(frame, (x, y), 50, (0, 0, 255), 15)
            cv.drawMarker(frame, (x, y), (0, 0, 255), cv.MARKER_CROSS, 40, 5)
            cv.putText(frame,
                       f"tan_x={self.tracked_object.tan_x:+.3f} tan_y={self.tracked_object.tan_y:+.3f}",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv.imshow("Stream Viewer", frame)
        cv.waitKey(1)

    def angle_to_pixel(self, tan_x, tan_y, w, h):
        # Inverse of pixel_to_angle in vision_processor.py

        x = w/2 + np.rad2deg(np.arctan(tan_x)) * (w / self.camera_fov_w)
        y = h/2 - np.rad2deg(np.arctan(tan_y)) * (h / self.camera_fov_h)

        return int(x), int(y)

def main(args=None):
    rclpy.init(args=args)

    streamViewerNode = StreamViewerNode()

    rclpy.spin(streamViewerNode)

    streamViewerNode.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
