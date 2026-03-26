from abc import ABC, abstractmethod
import threading
import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class VideoPipelineBase(ABC):
    def __init__(self):
        self.frame: np.ndarray | None = None
        self._frame_lock: threading.RLock = threading.RLock()

    def _set_frame(self, frame: np.ndarray) -> None:
        """Write a new frame."""
        with self._frame_lock:
            self.frame = frame

    def _get_frame(self) -> np.ndarray | None:
        """Read the current frame safely."""
        with self._frame_lock:
            return getattr(self, "frame", None)
        
    def get_frame(self) -> np.ndarray | None:
        return self._get_frame()

    @abstractmethod
    def run(self) -> None:
        pass

    def _sink_callback(self, sink) -> Gst.FlowReturn:
            sample = sink.emit("pull-sample")
            if sample is None:
                # Silently Fail
                return Gst.FlowReturn.OK

            buf = sample.get_buffer()
            caps = sample.get_caps().get_structure(0)
            width = caps.get_value("width")
            height = caps.get_value("height")

            ok, map_info = buf.map(Gst.MapFlags.READ)
            if not ok:
                # Silently Fail
                return Gst.FlowReturn.OK

            try:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3))
                self._set_frame(frame.copy())  # copy before unmap
            finally:
                buf.unmap(map_info)

            return Gst.FlowReturn.OK
