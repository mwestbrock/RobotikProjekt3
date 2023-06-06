#ObjektDetector Converts video frames to binary images and detects white objects in them.
#Only Detect
import cv2
import numpy as np

class ObjectDetector:

    def __init__(self, frame):
        """
        Initializes the ObjectDetector class.

        Args:
            frame (numpy.ndarray): The frame to be processed.
        """
        self.frame = frame
        self.binary_frame = self.convertToBinary()
        self.contours = self.getContours()
        self.boundingBoxes = self.getBoundingBoxes()
        self.objects = self.getObjects()

    def convertToBinary(self):
        """
        Converts the frame to a binary image.

        Returns:
            numpy.ndarray: The binary image.
        """
        threshold = 150
        gray_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        _, binary_frame = cv2.threshold(gray_frame, threshold, 255, cv2.THRESH_BINARY)
        return binary_frame

    def getContours(self):
        """
        Finds the contours in the binary image.

        Returns:
            list: The contours.
        """

        contours, _ = cv2.findContours(self.binary_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def getBoundingBoxes(self):
        """
        Finds the bounding boxes of the contours.

        Returns:
            list: The bounding boxes.
        """
        boundingBoxes = []
        for contour in self.contours:
            boundingBoxes.append(cv2.boundingRect(contour))
        return boundingBoxes

    def getObjects(self):
        """
        Finds the objects in the frame.

        Returns:
            list: The objects.
        """
        objects = []
        for boundingBox in self.boundingBoxes:
            x, y, w, h = boundingBox
            if w > 150 and h > 150:
                objects.append(boundingBox)
        return objects

    def drawBoundingBoxes(self):
        """
        Draws the bounding boxes of the objects in the frame.
        """
        for object in self.objects:
            x, y, w, h = object
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    def getCenterPoints(self):
        """
        Calculates the center points of the objects.

        Returns:
            list: The center points.
        """
        centerPoints = []
        for object in self.objects:
            x, y, w, h = object
            centerPoints.append((x + w / 2, y + h / 2))
        return centerPoints

    def drawBoundingBoxes(image, contours):
        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 4000:
                x, y, w, h = cv2.boundingRect(cnt)

                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                detections.append([x, y, w, h])
        return image, detections





