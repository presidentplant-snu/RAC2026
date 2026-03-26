from .pipelinebase import VideoPipelineBase

import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class GazeboPipeline(VideoPipelineBase):
    def __init__(self):
        VideoPipelineBase.__init__(self)

        pipeline_str = f"""
        udpsrc port=5600 ! 
        application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 !
        rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! 
        appsink emit-signals=true sync=false max-buffers=2 drop=true name=sink
        """

        self.pipeline = Gst.parse_launch(pipeline_str)
        appsink = self.pipeline.get_by_name("sink")

        appsink.connect("new-sample", self._sink_callback)


    def run(self) -> None:
        self.pipeline.set_state(Gst.State.PLAYING)

        context = GLib.MainContext.new()  # isolated context, not the default
        loop = GLib.MainLoop.new(context, False)
        context.push_thread_default()
        try:
            loop.run()
        finally:
            self.pipeline.set_state(Gst.State.NULL)
