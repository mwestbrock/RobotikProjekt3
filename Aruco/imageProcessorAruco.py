import cv2
import cv2.aruco as aruco
import numpy as np

class ImageProcessor:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_centers = []

    def detect_markers(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = aruco.ArucoDetector(self.aruco_dict, detectorParams=self.parameters)

        corners, ids, rejected_corners = detector.detectMarkers(gray)

        self.marker_centers = []

        if len(corners) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                center_x = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                center_y = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)
                self.marker_centers.append((ids[i][0], center_x, center_y))

        # Sortiere die marker_centers nach aufsteigender ID
        self.marker_centers = sorted(self.marker_centers, key=lambda x: x[0])

        cv2.imshow("Aruco Detection", frame)

        if len(self.marker_centers) >= 4:
            self.transform_to_birds_eye_view()

    def transform_to_birds_eye_view(self):
        ret, frame = self.cap.read()
        height, width, _ = frame.shape

        src_points = np.array((self.marker_centers[1][1:], self.marker_centers[0][1:],
                               self.marker_centers[2][1:], self.marker_centers[3][1:]), dtype=np.float32)
        dst_points = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
        perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        warped_frame = cv2.warpPerspective(frame, perspective_matrix, (width, height))
        cv2.imshow("Bird's-eye View", warped_frame)

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


# Usage
detector = ImageProcessor()

while True:
    detector.detect_markers()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

detector.release()
