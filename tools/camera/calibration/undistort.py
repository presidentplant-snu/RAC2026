import cv2
import numpy as np
from threading import Thread
import queue

# Camera calibration parameters (TODO: get decent calibration)
camera_matrix = np.array([
    [1.08376469e+03, 0.00000000e+00, 9.42911010e+02],
    [0.00000000e+00, 1.08412582e+03, 5.35392847e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)

dist_coeffs = np.array([
    [-0.11369982, 0.11115915, 0.00017042, -0.00018905, -0.01477928]
], dtype=np.float32)

# RTSP stream URL
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"

class RTSPReader:
    """Thread-safe RTSP frame reader"""
    def __init__(self, rtsp_url, queue_size=2):
        self.rtsp_url = rtsp_url
        self.queue = queue.Queue(maxsize=queue_size)
        self.thread = Thread(target=self._read_frames, daemon=True)
        self.running = False
        
    def start(self):
        self.running = True
        self.thread.start()
        
    def stop(self):
        self.running = False
        self.thread.join(timeout=5)
        
    def _read_frames(self):
        cap = cv2.VideoCapture(self.rtsp_url)
        
        if not cap.isOpened():
            print(f"Error: Cannot open RTSP stream: {self.rtsp_url}")
            self.running = False
            return
        
        print(f"RTSP stream opened successfully")
        
        # Get video properties
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Video properties: {width}x{height} @ {fps} FPS")
        
        while self.running:
            ret, frame = cap.read()
            
            if not ret:
                print("Error reading frame from RTSP stream")
                break
            
            # Drop frame if queue is full (keep latest frames)
            if self.queue.full():
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.queue.put(frame)
        
        cap.release()
        self.running = False
    
    def get_frame(self):
        try:
            return self.queue.get(timeout=1)
        except queue.Empty:
            return None


def main():
    print("Starting RTSP Undistortion Stream...")
    print(f"RTSP URL: {RTSP_URL}")
    
    # Initialize RTSP reader
    reader = RTSPReader(RTSP_URL)
    reader.start()
    
    # Precompute the undistortion maps for faster processing
    # These will be calculated once based on the first frame
    map_x = None
    map_y = None
    
    try:
        while True:
            frame = reader.get_frame()
            
            if frame is None:
                print("Waiting for frames...")
                continue
            
            # Initialize maps on first frame
            if map_x is None:
                h, w = frame.shape[:2]
                print(f"Frame size: {w}x{h}")
                
                # Create undistortion maps
                map_x, map_y = cv2.initUndistortRectifyMap(
                    camera_matrix,
                    dist_coeffs,
                    None,  # R (rectification matrix, None for no rectification)
                    None,  # newCameraMatrix (None to use camera_matrix)
                    (w, h),
                    cv2.CV_32F
                )
                print("Undistortion maps computed")
            
            # Apply undistortion using precomputed maps
            undistorted = cv2.remap(
                frame,
                map_x,
                map_y,
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )
            
            # Create overlay with transparency
            # Blend: undistorted on top of frame with 50% transparency
            t = 0.3
            overlay = cv2.addWeighted(frame, 1-t, undistorted, t, 0)
            
            # Resize for display if too large
            if overlay.shape[1] > 1920:
                scale = 1920 / overlay.shape[1]
                overlay = cv2.resize(overlay, None, fx=scale, fy=scale)
            
            cv2.imshow("Frame Overlay (Original 50% + Undistorted 50%)", overlay)
            
            # Press 'q' to quit, 's' to save frame
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('s'):
                cv2.imwrite("undistorted_frame.jpg", undistorted)
                print("Frame saved as 'undistorted_frame.jpg'")
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    finally:
        reader.stop()
        cv2.destroyAllWindows()
        print("Stream closed")


if __name__ == "__main__":
    main()
