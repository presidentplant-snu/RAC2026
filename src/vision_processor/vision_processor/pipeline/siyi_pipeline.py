from .pipelinebase import VideoPipelineBase

import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class SIYIPipeline(VideoPipelineBase):
    def __init__(self, target_ip: str = "srt://192.168.10.36:5000", hw_accel: bool = False) -> None:
        VideoPipelineBase.__init__(self)

        
        decoder = "v4l2h264dec" if hw_accel else "decodebin"

        # THIS IS WRONG! HAVE TO CHANGE TO h264 / change ip addr later on.
        pipeline_str = f"""
        rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! 
        rtph264depay ! h264parse config-interval=1 ! tee name=t
      
        t. ! queue max-size-buffers=2 ! {decoder} ! videoconvert ! video/x-raw,format=BGR ! 
        appsink emit-signals=true sync=false max-buffers=2 drop=true name=sink
        t. ! queue max-size-buffers=2 ! mpegtsmux alignment=7 ! srtsink uri="{target_ip}" mode=caller sync=false 
        """
        self.pipeline = Gst.parse_launch(pipeline_str)
        appsink = self.pipeline.get_by_name("sink")

        appsink.connect("new-sample", self._sink_callback)



    def run(self) -> None:
        self.pipeline.set_state(Gst.State.PLAYING)
        self._loop = GLib.MainLoop()
        try:
            self._loop.run()
        finally:
            self.pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    siyiPipeline = SIYIPipeline()
    siyiPipeline.init(target_ip = "srt://192.168.10.36:5000")
    siyiPipeline.run()
