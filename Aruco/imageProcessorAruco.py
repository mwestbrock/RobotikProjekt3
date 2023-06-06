import cv2
import cv2.aruco as aruco
import numpy as np

class ImageProcessor:
    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.warped_frame = None

    def resize_frame(self, frame, scale_percent):
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)

        return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = aruco.ArucoDetector(self.aruco_dict, detectorParams=self.parameters)

        corners, ids, rejected_corners = detector.detectMarkers(gray)

        marker_centers = []

        if len(corners) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                center_x = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                center_y = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)
                marker_centers.append((ids[i][0], center_x, center_y))

        # Sortiere die marker_centers nach aufsteigender ID
        marker_centers = sorted(marker_centers, key=lambda x: x[0])

        return marker_centers
        #cv2.imshow("Aruco Detection", frame)





    def transform_to_birds_eye_view(self, frame, marker_centers):
        height, width, _ = frame.shape

        src_points = np.array((marker_centers[2][1:], marker_centers[3][1:],
                               marker_centers[1][1:], marker_centers[0][1:]), dtype=np.float32)
        dst_points = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
        perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        warped_frame = cv2.warpPerspective(frame, perspective_matrix, (width, height))
        return warped_frame
        #cv2.imshow("Bird's-eye View", self.warped_frame)

    def crop_warped_image(self, warped_frame):
        if warped_frame is None:
            return None
        x = 0
        y = self.cmToPixel(3)
        width = warped_frame.shape[1]
        height = warped_frame.shape[0] - self.cmToPixel(6)

        cropped_frame = warped_frame[y:y+height, x:x+width]
        #cv2.imshow("Crop_IMG", self.cropped_frame)
        return cropped_frame

    def cmToPixel(self, cm):
        # 1cm = 40px
        return cm * 40

    def release(self):
        cv2.destroyAllWindows()

#Beispiel aufruf mit Video
cap = cv2.VideoCapture("C:/Users/Maximilian/Desktop/Video.mp4")