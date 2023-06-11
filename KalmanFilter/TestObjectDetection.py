from collections import deque
import cv2
import numpy as np
from KalmanFilter.kalmanfilter import KalmanFilter
from tracker import *
from imageProcessorAruco import ImageProcessor
from Greifpunkt import grippingPoint

class ObjectDetection:
    def __init__(self):
        self.kf = KalmanFilter()
        self.tracker = Tracker()
        self.threshold = 150
        self.imageProcessor = ImageProcessor()
        self.object_ids = []
        self.velocities = []
        self.object_queue = {}
        self.minItemArea = 4000
        self.minDetectioBorder = 0
        self.maxDetectionBorder = 40 * 10


    def process_frame(self, cropped_frame, fps):

        gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detectionsArr = []
        predictedArr = []
        velocitiesArr = []
        object_idsArr = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.minItemArea:
                x, y, w, h = cv2.boundingRect(cnt)

                if x > self.minDetectioBorder and x + w < self.maxDetectionBorder:
                    bounding_box = thresh[y:y + h, x:x + w]
                    detectionsArr.append([x, y, w, h])
                    gpx, gpy = grippingPoint(bounding_box)
                    cx = x + gpx
                    cy = y + gpy
                    cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    predicted = self.kf.predict(cx, cy)
                    cv2.circle(cropped_frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                    cv2.circle(cropped_frame, (int(predicted[0]), int(predicted[1])), 7, (0, 255, 0), -1)
                    predictedArr.append(predicted)

        boxes_ids = self.tracker.update(detectionsArr)

        for box_id in boxes_ids:
            x, y, w, h, id = box_id
            cv2.putText(cropped_frame, str(id), (x, y + 30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            object_idsArr.append(id)
            object_velocity = self.tracker.velocity(id, fps)
            velocitiesArr.append(object_velocity)
            filtered_velocities = [v for v in velocitiesArr if v is not None]
            if len(filtered_velocities) > 0:
                avg_velocity = sum(filtered_velocities) / len(filtered_velocities)
                avg_velocity = np.round(avg_velocity, 2)
                cv2.putText(cropped_frame, str(avg_velocity) + "cm/s", (x, y + 150), cv2.FONT_HERSHEY_PLAIN, 2,
                            (0, 100, 255), 2)

        return predictedArr, velocitiesArr, object_idsArr

    def create_object_queue(self, object_id, predicted, velocities):
        for i in range(len(object_id)):
            if object_id[i] not in self.object_ids:
                self.object_ids.append(object_id[i])
                self.object_queue[object_id[i]] = deque(maxlen=30)
            self.object_queue[object_id[i]].appendleft((predicted[i], velocities[i]))

        return self.object_queue

    def main(self):
        kf = KalmanFilter()
        tracker = Tracker()
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

            predicted, velocities, object_id = self.process_frame(cropped_frame, fps)

            object_queue = self.create_object_queue(object_id, predicted, velocities)
            print(object_queue)

            cv2.imshow("Frame", cropped_frame)
            cv2.waitKey(wait_time)

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ObjectDetection().main()
