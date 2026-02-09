#!/bin/bash
gst-launch-1.0 rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! \
  rtph265depay ! h265parse ! v4l2h265dec ! \
  videoscale ! video/x-raw,width=1280,height=720 ! \
  v4l2h265enc extra-controls="controls,video_bitrate=3000000,hevc_i_frame_qp_value=32,hevc_p_frame_qp_value=34,hevc_b_frame_qp_value=36,hevc_minimum_qp_value=30,hevc_maximum_qp_value=42,hevc_i_frame_minimum_qp_value=30,hevc_i_frame_maximum_qp_value=42,hevc_p_frame_minimum_qp_value=32,hevc_p_frame_maximum_qp_value=42,video_gop_size=10,intra_refresh_period=5,intra_refresh_period_type=0;" ! \
  h265parse ! srtsink uri=srt://192.168.10.36:5000 latency=100 mode=caller sync=false
