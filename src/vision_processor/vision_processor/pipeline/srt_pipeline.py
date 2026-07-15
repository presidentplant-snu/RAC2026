from .pipelinebase import VideoPipelineBase

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class SRTPipeline(VideoPipelineBase):
    def __init__(self, listen_uri: str = "srt://:5000", latency: int = 100) -> None:
        VideoPipelineBase.__init__(self)

        self.latency = latency

        pipeline_str = f"""
        srtsrc uri={listen_uri} mode=listener keep-listening=true latency={latency} !
        queue ! tsdemux ! h265parse ! decodebin ! videoconvert ! video/x-raw,format=BGR !
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
