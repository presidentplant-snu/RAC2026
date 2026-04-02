"""
RTSP video capture with AprilTag detection using OpenCV
"""

import cv2
import sys

def detect_apriltags():
    """Capture video from RTSP URL and detect AprilTags"""
    
    detector = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h10)
    params = cv2.aruco.DetectorParameters()
    apriltag_detector = cv2.aruco.ArucoDetector(detector, params)

    rtsp = "rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! \
  rtph265depay ! h265parse ! decodebin ! videoconvert ! appsink"

    hdmi = "v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=60/1 ! jpegdec ! video/x-raw,format=I420 ! videoconvert ! appsink"

    cap = cv2.VideoCapture(rtsp, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        return
    
    print("Press 'q' to quit\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame")
            break
        
        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags
        corners, ids, rejected = apriltag_detector.detectMarkers(gray)
        
        # Draw detections
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"Detected {len(ids)} AprilTag(s): {ids.flatten().tolist()}")
        
        # Display frame
        cv2.imshow("AprilTag Detection", frame)
        
        # Check for quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_apriltags()
