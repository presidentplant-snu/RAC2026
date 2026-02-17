#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import time
import json

Gst.init(None)

pipeline_str = """
  mpegtsmux name=mux ! srtsink uri="srt://192.168.10.36:5000" mode=caller sync=false 
  rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! 
    rtph265depay ! h265parse config-interval=1 ! mux.
  appsrc name=textsrc caps="meta/x-klv,parsed=true" format=time do-timestamp=true ! queue ! mux.
"""

pipeline = Gst.parse_launch(pipeline_str)
appsrc = pipeline.get_by_name('textsrc')


counter = 0
duration = 33

def push_text():
    global counter
    
    text_data = json.dumps({
        'timestamp': time.time(),
        'message': 'Hello from KLV',
        'counter': counter
    }).encode('utf-8')
    
    buffer = Gst.Buffer.new_wrapped(text_data)
    
    timestamp = counter * duration * Gst.MSECOND
    buffer.pts = timestamp
    buffer.dts = timestamp
    buffer.duration = duration * Gst.MSECOND

    ret = appsrc.emit('push-buffer', buffer)
    
    if ret != Gst.FlowReturn.OK:
        print(f"Error pushing buffer: {ret}")

    counter += 1
    
    return True

pipeline.set_state(Gst.State.PLAYING)
GLib.timeout_add(duration, push_text)  # Push every 100ms

loop = GLib.MainLoop()
try:
    print("Sender running... Press Ctrl+C to stop")
    loop.run()
except KeyboardInterrupt:
    print("\nStopping sender...")
    pipeline.set_state(Gst.State.NULL)
