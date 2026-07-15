import os
# Decouple Qt (used by cv2.imshow) from the GLib event loop. GStreamer runs its
# own GLib MainLoop in a worker thread; without this, Qt's GLib event dispatcher
# fights it, producing "Timers cannot be started from another thread" and
# "g_main_context_push_thread_default: assertion 'acquired_context' failed".
# Must be set before Qt is loaded (i.e. before importing cv2 / gi).
os.environ.setdefault("QT_NO_GLIB", "1")

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

from .detectors import ArucoDetector, OnnxDetector
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
            self.pipeline = SIYIPipeline(source_uri=self.siyi_uri,
                                         target_ip=self.srt_uri, hw_accel=self.hw_accel)
        
        self.pipeline_thread = threading.Thread(target=self.pipeline.run, daemon=True)
        self.pipeline_thread.start()

        self.aruco_detector = ArucoDetector()

        # Patient detection via ONNX (opt-in). Empty path -> disabled.
        self.patient_detector = None
        if self.patient_model_path:
            self.patient_detector = OnnxDetector(self.patient_model_path)
            self.get_logger().info(
                f"Loaded patient ONNX model: {self.patient_model_path}")

    def setup_ros2(self):
        self.aircraft_state_subscriber = self.create_subscription(
                AircraftState, '/aircraft_state',
                self._aircraft_state_callback,
                QoSPresetProfiles.SERVICES_DEFAULT.value
                )

        self.tracked_object_publisher = self.create_publisher(
                CameraTrackedObject, '/camera_tracked_object',
                QoSPresetProfiles.SERVICES_DEFAULT.value
                )

        # Set to 10 Hz, might change later depending on detection speed.
        self.main_timer = self.create_timer(0.10, self._main_timer_callback)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_gazebo', False),
                ('show_debug', False),
                ('patient_model_path', ''),
                # RTSP source the SIYI pipeline pulls the camera stream from.
                ('siyi_uri', 'rtsp://192.168.144.25:8554/main.264'),
                # Use the hardware H.265 decoder (v4l2h265dec) in the SIYI
                # pipeline; set false to fall back to software decodebin.
                ('hw_accel', True),
                # SRT endpoint the SIYI pipeline re-streams to (srtsink, caller);
                # the stream_viewer reads the same URI as its srtsrc listener.
                ('srt_uri', 'srt://192.168.10.36:5000'),
                # Camera geometry. FOVs are the full angular field of view in
                # degrees; offsets are the camera position relative to the
                # vehicle CG in meters (passed through to the tracked object).
                ('camera_width', 1920),
                ('camera_height', 1080),
                ('camera_fov_w', 80.0),
                ('camera_fov_h', 45.0),
                ('camera_offset_x', 0.0),
                ('camera_offset_y', 0.0),
            ]
        )

        self.use_gazebo = self.get_parameter('use_gazebo').value
        self.show_debug = self.get_parameter('show_debug').value
        self.patient_model_path = self.get_parameter('patient_model_path').value
        self.siyi_uri = self.get_parameter('siyi_uri').value
        self.hw_accel = self.get_parameter('hw_accel').value
        self.srt_uri = self.get_parameter('srt_uri').value

        self.camera_w = self.get_parameter('camera_width').value
        self.camera_h = self.get_parameter('camera_height').value
        self.camera_fov_w = self.get_parameter('camera_fov_w').value
        self.camera_fov_h = self.get_parameter('camera_fov_h').value
        self.camera_offset_x = self.get_parameter('camera_offset_x').value
        self.camera_offset_y = self.get_parameter('camera_offset_y').value
        

    def _aircraft_state_callback(self, msg):
        self.aircraft_state = msg

    def _main_timer_callback(self):
        start_time = 0
        end_time = 0

        if self.aircraft_state is None:
            self.get_logger().warn("No aircraft state received yet",
                                   throttle_duration_sec=1.0)
            return

        self.frame = self.pipeline.get_frame()
        if self.frame is None:
            self.get_logger().warn("No frame available from pipeline",
                                   throttle_duration_sec=1.0)
            return

        if self.show_debug:
            start_time = time.perf_counter()

        detection = None
        marker_color = None
        match self.aircraft_state.track_type:
            case AircraftState.TRACK_PATIENT:
                if self.patient_detector is not None:
                    detection = self.patient_detector.detect(self.frame)
                    marker_color = (255, 0, 0)
            case AircraftState.TRACK_ARUCO:
                detection = self.aruco_detector.detect(self.frame)
                marker_color = (0, 0, 255)
            case _:
                pass

        # Build a fresh message every cycle so we never republish a stale hit.
        # A live detection carries the target angle; otherwise publish NaN so the
        # downstream tracker invalidates its target instead of chasing a ghost.
        self.tracked_object = CameraTrackedObject()
        self.tracked_object.offset_x = self.camera_offset_x
        self.tracked_object.offset_y = self.camera_offset_y
        if detection is not None:
            self.tracked_object.tan_x, self.tracked_object.tan_y = \
                    self.pixel_to_angle(detection[0], detection[1])
        else:
            self.tracked_object.tan_x = float('nan')
            self.tracked_object.tan_y = float('nan')

        if self.show_debug:
            end_time = time.perf_counter()
            execution_time = end_time - start_time
            self.get_logger().info(
                        f"Detection took {execution_time:.5f}s",
                        throttle_duration_sec=1.0
                    )
            # Draw on a copy: the pipeline hands out a reference to its stored
            # frame, and the next detection may run on the same frame again.
            debug_frame = self.frame.copy()
            if detection is not None and marker_color is not None:
                cv.circle(debug_frame, (int(detection[0]), int(detection[1])),
                          50, marker_color, 15)
            cv.imshow("Debug", debug_frame)
            cv.waitKey(1)

        self.tracked_object_publisher.publish(self.tracked_object)




    def shutdown(self):
        """Tear down the video pipeline and close debug windows."""
        pipeline = getattr(self, "pipeline", None)
        if pipeline is not None:
            pipeline.stop()
        thread = getattr(self, "pipeline_thread", None)
        if thread is not None:
            # Wait for run()'s finally to set the pipeline to NULL (RTSP TEARDOWN).
            thread.join(timeout=2.0)
        cv.destroyAllWindows()

    def pixel_to_angle(self, x, y):
        # Up / Right is +ve. Pinhole (rectilinear) model: a standard lens is
        # linear in tan(angle), not in angle, so map pixel offset directly to a
        # tangent via the focal length fx = (w/2) / tan(fov_w/2).
        fx = (self.camera_w / 2) / np.tan(np.deg2rad(self.camera_fov_w) / 2)
        fy = (self.camera_h / 2) / np.tan(np.deg2rad(self.camera_fov_h) / 2)

        tan_x = (x - self.camera_w / 2) / fx
        tan_y = -(y - self.camera_h / 2) / fy

        # Cast to Python float: NumPy scalars (np.float32 under NEP 50) are not
        # float subclasses and fail the generated ROS setter's isinstance check.
        return float(tan_x), float(tan_y)

def main(args=None):
    rclpy.init(args=args)

    visionProcessorNode = VisionProcessorNode()

    try:
        rclpy.spin(visionProcessorNode)
    except KeyboardInterrupt:
        pass
    finally:
        visionProcessorNode.shutdown()
        visionProcessorNode.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

