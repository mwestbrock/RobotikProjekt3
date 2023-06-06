import cv2
import cv2.aruco as aruco
import numpy as np

class ImageProcessor:
    def __init__(self):
        """
        Initializes the ImageProcessor class with the necessary variables.
        """
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.warped_frame = None

    def resize_frame(self, frame, scale_percent):
        """
        Resizes the input frame by a given scale percentage.

        Args:
            frame (numpy.ndarray): The input frame to be resized.
            scale_percent (int): The scale percentage for resizing.

        Returns:
            numpy.ndarray: The resized frame.
        """
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)

        return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

    def detect_markers(self, frame):
        """
        Detects ArUco markers in the input frame.

        Args:
            frame (numpy.ndarray): The input frame to detect markers from.

        Returns:
            list: A list of tuples containing marker IDs and their center coordinates.
        """
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

        # Sort the marker_centers in ascending order based on ID
        marker_centers = sorted(marker_centers, key=lambda x: x[0])

        return marker_centers

    def transform_to_birds_eye_view(self, frame, marker_centers):
        """
        Transforms the input frame to a bird's-eye view using perspective transformation.

        Args:
            frame (numpy.ndarray): The input frame to transform.
            marker_centers (list): A list of tuples containing marker IDs and their center coordinates.

        Returns:
            numpy.ndarray: The transformed frame in bird's-eye view.
        """
        height = 450
        width = 450

        src_points = np.array((marker_centers[2][1:], marker_centers[3][1:],
                               marker_centers[1][1:], marker_centers[0][1:]), dtype=np.float32)
        dst_points = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
        perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        warped_frame = cv2.warpPerspective(frame, perspective_matrix, (width, height))
        # Draw lines for conveyor belt
        cv2.line(warped_frame, (0, self.cmToPixel(3)), (width, self.cmToPixel(3)), (0, 0, 255), 2)
        cv2.line(warped_frame, (self.cmToPixel(3), 0), (self.cmToPixel(3), height), (0, 0, 255), 2)
        return warped_frame

    def crop_warped_image(self, warped_frame):
        """
        Crops the warped frame to remove unwanted regions.

        Args:
            warped_frame (numpy.ndarray): The warped frame to be cropped.

        Returns:
            numpy.ndarray: The cropped frame.
        """
        if warped_frame is None:
            return None
        x = 0
        y = self.cmToPixel(3)
        width = warped_frame.shape[1]
        height = warped_frame.shape[0] - self.cmToPixel(6)

        cropped_frame = warped_frame[y:y+height, x:x+width]
        return cropped_frame

    def cmToPixel(self, cm):
        """
        Converts centimeters to pixels.

        Args:
            cm (float): The value in centimeters.

        Returns:
            int: The equivalent value in pixels.
        """
        # 1cm = 40px
        return cm * 40

    def release(self):
        """
        Closes all open windows.
        """
        cv2.destroyAllWindows()

# Example usage with a video
cap = cv2.VideoCapture("F:/mariu/Desktop/Projekt 3 HS/Videos/AruCoGemischt.mp4")
imageProcessor = ImageProcessor()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = imageProcessor.resize_frame(frame, 50)
    marker_centers = imageProcessor.detect_markers(frame)
    warped_frame = imageProcessor.transform_to_birds_eye_view(frame, marker_centers)
    cropped_frame = imageProcessor.crop_warped_image(warped_frame)
    cv2.imshow("Frame", warped_frame)
    cv2.imshow("Cropped", cropped_frame)
    cv2.waitKey(1)
imageProcessor.release()
cap.release()
cv2.destroyAllWindows()
