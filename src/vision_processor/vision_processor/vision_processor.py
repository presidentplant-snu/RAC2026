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
            self.pipeline = SIYIPipeline(target_ip="srt://192.168.10.36:5000", hw_accel=True)
        
        self.pipeline_thread = threading.Thread(target=self.pipeline.run, daemon=True)
        self.pipeline_thread.start()

        self.camera_w = 1920
        self.camera_h = 1080
        self.camera_fov_w = 80 # degrees
        self.camera_fov_h = 80*9/16 # degrees

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
                ('patient_model_path',
                 '/home/radxa/RAC2026/model/yolo26n_1280_3out_int8.onnx')
            ]
        )

        self.use_gazebo = self.get_parameter('use_gazebo').value
        self.show_debug = self.get_parameter('show_debug').value
        self.patient_model_path = self.get_parameter('patient_model_path').value
        

    def _aircraft_state_callback(self, msg):
        self.aircraft_state = msg

    def _main_timer_callback(self):
        start_time = 0
        end_time = 0

        if self.aircraft_state is None:
            print("no state")
            return

        self.frame = self.pipeline.get_frame() 
        if self.frame is None:
            print("no frame")
            return

        if self.show_debug:
            start_time = time.perf_counter()

        match self.aircraft_state.track_type:
            case AircraftState.TRACK_PATIENT:
                if self.patient_detector is not None:
                    detection = self.patient_detector.detect(self.frame)
                    if detection is not None:
                        self.tracked_object = CameraTrackedObject()
                        cv.circle(self.frame, (int(detection[0]), int(detection[1])), 50, (255, 0, 0), 15)
                        self.tracked_object.tan_x, self.tracked_object.tan_y = \
                                self.pixel_to_angle(detection[0], detection[1])
            case AircraftState.TRACK_ARUCO:
                detection = self.aruco_detector.detect(self.frame)
                if detection is not None:
                    self.tracked_object  = CameraTrackedObject()
                    cv.circle(self.frame, (int(detection[0]), int(detection[1])), 50, (0, 0, 255), 15)
                    self.tracked_object.tan_x, self.tracked_object.tan_y = \
                            self.pixel_to_angle(detection[0], detection[1])
            case _:
                pass

        if self.show_debug:
            end_time = time.perf_counter()
            execution_time = end_time - start_time
            self.get_logger().info(
                        f"Detection took {execution_time:.5f}s",
                        throttle_duration_sec=1.0
                    )
            cv.imshow("Debug", self.frame)
            cv.waitKey(1)

        if self.tracked_object is None:
            return

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
        # Up / Right is +ve

        angle_x = np.tan(np.deg2rad((x-self.camera_w/2) * (self.camera_fov_w / self.camera_w)))
        angle_y = -np.tan(np.deg2rad((y-self.camera_h/2) * (self.camera_fov_h / self.camera_h)))
        
        return angle_x, angle_y

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

