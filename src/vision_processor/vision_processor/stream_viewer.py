import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from aircraft_msgs.msg import CameraTrackedObject

import math
import threading
import time
from collections import deque
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

        # Detections arrive ~real-time, but the SRT stream is buffered by
        # srt_latency ms. Hold detections in this queue and only overlay them
        # once srt_latency has elapsed, so markers line up with the video.
        self.pending_objects: deque[tuple[float, CameraTrackedObject]] = deque()

        self.setup_ros2()

        self.pipeline = SRTPipeline(listen_uri=self.srt_uri, latency=self.srt_latency)
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
                ('srt_uri', 'srt://192.168.10.36:5000'),
                ('srt_latency', 100),
                ('marker_timeout', 1.0),
                # Must match the camera FOV parameters in vision_processor so the
                # overlaid markers line up with the detections.
                ('camera_fov_w', 80.0),
                ('camera_fov_h', 45.0),
            ]
        )

        self.srt_uri = self.get_parameter('srt_uri').value
        self.srt_latency = self.get_parameter('srt_latency').value
        self.marker_timeout = self.get_parameter('marker_timeout').value
        self.camera_fov_w = self.get_parameter('camera_fov_w').value
        self.camera_fov_h = self.get_parameter('camera_fov_h').value

        # srtsrc latency is in milliseconds; convert to seconds for scheduling.
        self.srt_latency_sec = self.srt_latency / 1000.0

    def _tracked_object_callback(self, msg):
        # tan_x is NaN when the processor had no detection: drop it so a recent
        # valid marker stays on screen until marker_timeout expires.
        if math.isnan(msg.tan_x):
            return
        # Defer overlay until the matching (latency-buffered) frame is shown.
        self.pending_objects.append((time.monotonic(), msg))

    def _release_due_objects(self):
        # Promote detections whose matching video frame should now be visible,
        # i.e. those received at least srt_latency ago.
        now = time.monotonic()
        while self.pending_objects and \
                now - self.pending_objects[0][0] >= self.srt_latency_sec:
            recv_time, msg = self.pending_objects.popleft()
            self.tracked_object = msg
            self.tracked_object_time = recv_time + self.srt_latency_sec

    def _display_timer_callback(self):
        self._release_due_objects()

        frame = self.pipeline.get_frame()
        if frame is None:
            return

        frame = frame.copy()

        # tan_x is NaN when the processor had no detection — nothing to overlay.
        if self.tracked_object is not None and \
                not math.isnan(self.tracked_object.tan_x) and \
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
        # Inverse of pixel_to_angle in vision_processor.py (pinhole model):
        # pixel = center + tan * fx, with fx = (w/2) / tan(fov_w/2).
        fx = (w / 2) / np.tan(np.deg2rad(self.camera_fov_w) / 2)
        fy = (h / 2) / np.tan(np.deg2rad(self.camera_fov_h) / 2)

        x = w/2 + tan_x * fx
        y = h/2 - tan_y * fy

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
