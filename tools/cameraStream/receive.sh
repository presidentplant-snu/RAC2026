#!/bin/bash
gst-launch-1.0 srtsrc uri=srt://:5000 mode=listener keep-listening=true latency=150 ! \
  h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false
