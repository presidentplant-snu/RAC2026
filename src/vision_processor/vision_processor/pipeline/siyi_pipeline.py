from .pipelinebase import VideoPipelineBase

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class SIYIPipeline(VideoPipelineBase):
    def __init__(self, source_uri: str = "rtsp://192.168.144.25:8554/main.264",
                 target_ip: str = "srt://192.168.10.36:5000", hw_accel: bool = False) -> None:
        VideoPipelineBase.__init__(self)

        decoder = "v4l2h265dec" if hw_accel else "decodebin"

        # SIYI main stream is H.265. One branch decodes to BGR frames for
        # detection; the other re-muxes to MPEG-TS and re-streams over SRT
        # (srtsink, caller) for the stream_viewer, which parses h265.
        pipeline_str = f"""
        rtspsrc location={source_uri} latency=0 !
        rtph265depay ! h265parse config-interval=1 ! tee name=t

        t. ! queue leaky=downstream max-size-buffers=2 ! {decoder} ! videoconvert ! video/x-raw,format=BGR !
        appsink emit-signals=true sync=false max-buffers=2 drop=true name=sink
        t. ! queue leaky=downstream max-size-buffers=2 ! mpegtsmux alignment=7 ! srtsink uri="{target_ip}" mode=caller sync=false
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
    Gst.init(None)
    siyiPipeline = SIYIPipeline(source_uri="rtsp://192.168.144.25:8554/main.264",
                                target_ip="srt://192.168.10.36:5000")
    siyiPipeline.run()
