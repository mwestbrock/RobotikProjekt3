import math
import time
from collections import deque
import cv2
import numpy as np
from imageProcessorAruco import ImageProcessor
from Greifpunkt import grippingPoint
from filterpy.kalman import KalmanFilter
class DetectedObject:
    def __init__(self, id, x, y, timeInSeconds):
        self.id = id
        self.kalman = KalmanFilter(dim_x=4, dim_z=2)
        self.timeInSeconds = timeInSeconds

        self.kalman.x = np.array([x, y, 0., 0.])

        self.kalman.F = np.array([[1., 0., 1., 0.],
                                    [0., 1., 0., 1.],
                                    [0., 0., 1., 0.],
                                    [0., 0., 0., 1.]])

        self.kalman.H = np.array([[1., 0., 0., 0.],
                                    [0., 1., 0., 0.]])

        self.kalman.P = np.array([[1000., 0., 0., 0.],
                                    [0., 1000., 0., 0.],
                                    [0., 0., 1000., 0.],
                                    [0., 0., 0., 1000.]])

        self.kalman.R = np.array([[0.1, 0.],
                                    [0., 0.1]])

        self.kalman.Q = np.array([[0.001, 0., 0.001, 0.],
                                    [0., 0.001, 0., 0.001],
                                    [0.001, 0., 0.001, 0.],
                                    [0., 0.001, 0., 0.001]])






class ObjectDetection:
    def __init__(self):

        self.threshold = 150
        self.object_ids = []
        self.object_queue = {}
        self.minItemArea = 4000
        self.minDetectionBorder = 0
        self.maxDetectionBorder = 40 * 10
        self.detectedObjects = []
        self.id = 0


    def process_frame(self, cropped_frame, fps):

        gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.minItemArea:
                x, y, w, h = cv2.boundingRect(cnt)

                if x > self.minDetectionBorder and x + w < self.maxDetectionBorder:
                    bounding_box = thresh[y:y + h, x:x + w]
                    gpx, gpy = grippingPoint(bounding_box)
                    cx = x + gpx
                    cy = y + gpy

                    matchedObject = None
                    #Hier Könnte ihre Kalman Logik stehen
                    if self.detectedObjects:
                        obj = self.detectedObjects[-1]

                        dist = math.hypot(cx - obj.kalman.x[0], cy - obj.kalman.x[1])


                        if dist < 25:
                            matchedObject = obj

                    if matchedObject is not None:
                        matchedObject.kalman.update(np.array([cx, cy]))
                        predicted = matchedObject.kalman.predict(time.time()-matchedObject.timeInSeconds)

                        #cv2.circle(cropped_frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                        #cv2.circle(cropped_frame, (int(predicted[0]), int(predicted[1])), 7, (0, 255, 0), -1)

                        matchedObject.timeInSeconds = time.time()
                    else:
                        newObject = DetectedObject(++self.id, cx, cy, time.time())
                        self.detectedObjects.append(newObject)


















            """object_velocity = self.tracker.velocity(id, fps)
            velocitiesArr.append(object_velocity)
            filtered_velocities = [v for v in velocitiesArr if v is not None]
            if len(filtered_velocities) > 0:
                avg_velocity = sum(filtered_velocities) / len(filtered_velocities)
                avg_velocity = np.round(avg_velocity, 2)
                cv2.putText(cropped_frame, str(avg_velocity) + "cm/s", (x, y + 150), cv2.FONT_HERSHEY_PLAIN, 2,
                            (0, 100, 255), 2)"""

    def predict_timer(self):
        """Predicts the position of each object using kalman filter and timer"""
        # Predict position of each object
        for obj in self.detectedObjects:
            # Get current time in seconds
            timeInSeconds = time.time()
            # Check if 0.1 seconds have passed
            if timeInSeconds - obj.timeInSeconds > 0.1:
                # Update timer
                obj.timeInSeconds = timeInSeconds
                # Predict position
                obj.kalman.predict(timeInSeconds - obj.timeInSeconds)



    def create_object_queue(self, object_id, predicted):
        for i in range(len(object_id)):
            if object_id[i] not in self.object_ids:
                self.object_ids.append(object_id[i])
                self.object_queue[object_id[i]] = deque(maxlen=30)
            self.object_queue[object_id[i]].appendleft((predicted[i]))

        return self.object_queue

    def main(self):
        imageProcessor = ImageProcessor()
        object_ids = []

        cap = cv2.VideoCapture("F:/mariu/Desktop/Projekt 3 HS/Videos/AruCoGemischt.mp4")
        fps = cap.get(cv2.CAP_PROP_FPS)
        wait_time = int(1000 / fps)

        while True:
            ret, frame = cap.read()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            resized_frame = imageProcessor.resize_frame(frame, 100)
            marker_centers = imageProcessor.detect_markers(resized_frame)
            warped_frame = imageProcessor.transform_to_birds_eye_view(resized_frame, marker_centers)
            cropped_frame = imageProcessor.crop_warped_image(warped_frame)

            self.process_frame(cropped_frame, fps)

            #object_queue = self.create_object_queue(object_id, predicted)
            #print(object_queue)

            cv2.imshow("Frame", cropped_frame)
            cv2.waitKey(wait_time)

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ObjectDetection().main()
