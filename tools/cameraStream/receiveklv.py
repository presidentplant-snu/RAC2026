#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import json

Gst.init(None)

sample_count = 0

def on_pad_added(element, pad):
    pad_name = pad.get_name()
    print(f"New pad added: {pad_name}")
    
    if pad_name.startswith('video_'):
        video_queue = pipeline.get_by_name('video_queue')
        video_pad = video_queue.get_static_pad('sink')
        if not video_pad.is_linked():
            pad.link(video_pad)
            print("Linked video pad")
    
    elif pad_name.startswith('private_'):
        meta_queue = pipeline.get_by_name('meta_queue')
        meta_pad = meta_queue.get_static_pad('sink')
        if not meta_pad.is_linked():
            pad.link(meta_pad)
            print("Linked KLV metadata pad")

def on_new_sample(sink):
    global sample_count
    sample = sink.emit('pull-sample')
    if sample is None:
        return Gst.FlowReturn.OK
    
    buffer = sample.get_buffer()
    success, map_info = buffer.map(Gst.MapFlags.READ)
    
    if success:
        try:
            data = bytes(map_info.data)
            json_data = json.loads(data.decode('utf-8'))
            sample_count += 1
            print(f"[{sample_count}] Received: {json_data}")
        except Exception as e:
            print(f"Error decoding: {e}, raw: {data[:100]}")
        finally:
            buffer.unmap(map_info)
    
    return Gst.FlowReturn.OK

def on_message(bus, message):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"Error: {err}, {debug}")
        loop.quit()
    elif t == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(f"Warning: {err}")
    return True

pipeline_str = """
  srtsrc uri=srt://:5000 mode=listener keep-listening=true latency=400 !
    queue max-size-time=10000000 ! tsdemux name=demux
  queue name=video_queue ! h265parse ! decodebin ! autovideosink sync=false
  queue name=meta_queue leaky=1 ! 
    appsink name=textsink emit-signals=true sync=false drop=false max-buffers=0
"""

pipeline = Gst.parse_launch(pipeline_str)

demux = pipeline.get_by_name('demux')
demux.connect('pad-added', on_pad_added)

appsink = pipeline.get_by_name('textsink')
appsink.connect('new-sample', on_new_sample)

bus = pipeline.get_bus()
bus.add_signal_watch()
bus.connect("message", on_message)

pipeline.set_state(Gst.State.PLAYING)

loop = GLib.MainLoop()
try:
    print("Receiver running... Press Ctrl+C to stop")
    loop.run()
except KeyboardInterrupt:
    print("\nStopping receiver...")
    pipeline.set_state(Gst.State.NULL)
