import numpy as np
import cv2 as cv

class ArucoDetector():
    def __init__(self):

        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h10)
        self.params = cv.aruco.DetectorParameters_create()


        self.params.minMarkerPerimeterRate = 0.02

        # Performance optimizations
        self.params.adaptiveThreshWinSizeMin = 3   
        self.params.adaptiveThreshWinSizeMax = 20
        self.params.adaptiveThreshWinSizeStep = 4 
        
        self.params.maxErroneousBitsInBorderRate = 0.5

        self.params.polygonalApproxAccuracyRate = 0.08

        self.params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_NONE

        self.params.perspectiveRemovePixelPerCell = 12
        self.params.perspectiveRemoveIgnoredMarginPerCell = 0.2

        # Error correction
        self.params.errorCorrectionRate = 0.8  # Allow some error correction
        
        # Marker border
        self.params.markerBorderBits = 1  # Standard border width

    def detect(self, image: np.ndarray | None) -> np.ndarray | None:
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        corners, ids, _ =  cv.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        
        tag_center = None

        if ids is not None and len(ids) > 0:
            img_pts = corners[0].reshape(-1, 2).astype(np.float32)
            tag_center = np.mean(img_pts, axis=0)

        return tag_center
