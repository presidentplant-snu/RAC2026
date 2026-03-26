from .pipelinebase import VideoPipelineBase

import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class SIYIPipeline(VideoPipelineBase):
    def __init__(self, target_ip: str = "srt://192.168.10.36:5000", hw_accel: bool = False) -> None:
        VideoPipelineBase.__init__(self)

        
        decoder = "v4l2h265dec" if hw_accel else "decodebin"

        pipeline_str = f"""
      rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! 
        rtph265depay ! h265parse config-interval=1 ! tee name=t
        t. ! queue max-size-buffers=2 !  mpegtsmux ! srtsink uri="{target_ip}" mode=caller sync=false 
        t. ! queue max-size-buffers=2 ! {decoder} ! videoconvert ! video/x-raw,format=BGR ! 
        appsink emit-signals=true sync=false max-buffers=2 drop=true name=sink
        """

        self.pipeline = Gst.parse_launch(pipeline_str)
        appsink = self.pipeline.get_by_name("sink")

        appsink.connect("new-sample", self._sink_callback)



    def run(self) -> None:
        self.pipeline.set_state(Gst.State.PLAYING)
        loop = GLib.MainLoop()
        try:
            loop.run()
        finally:
            self.pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    siyiPipeline = SIYIPipeline()
    siyiPipeline.init(target_ip = "srt://192.168.10.36:5000")
    siyiPipeline.run()
