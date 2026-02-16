#!/bin/bash
gst-launch-1.0 rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! \
  rtph265depay ! h265parse config-interval= 1 ! \
  mpegtsmux ! \
  srtsink uri="srt://192.168.10.36:5000" mode=caller sync=false #auto-reconnect=false
