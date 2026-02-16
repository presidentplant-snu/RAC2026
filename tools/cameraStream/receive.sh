#!/bin/bash
gst-launch-1.0 srtsrc uri=srt://:5000 mode=listener keep-listening=true latency=400 ! queue ! \
   tsdemux ! h265parse ! decodebin !  autovideosink sync=false
