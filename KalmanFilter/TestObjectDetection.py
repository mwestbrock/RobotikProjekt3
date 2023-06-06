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
        self.imageProcessor = ImageProcessor()
        self.object_ids = []



    def detect_objects(cropped_frame, kf):
        gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        schwellenwert = 150
        ret, thresh = cv2.threshold(gray, schwellenwert, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        velocities = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 4000:
                x, y, w, h = cv2.boundingRect(cnt)

                if x > 0 and x + w < 40*10:
                    bounding_box = thresh[y:y+h, x:x+w]
                    detections.append([x, y, w, h])
                    gpx, gpy = grippingPoint(bounding_box)
                    cx = x + gpx
                    cy = y + gpy
                    cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    predicted = kf.predict(cx, cy)
                    cv2.circle(cropped_frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                    cv2.circle(cropped_frame, (int(predicted[0]), int(predicted[1])), 7, (0, 255, 0), -1)
                    print(predicted)

        return detections, velocities

    def object_tracking(cropped_frame, tracker, detections, fps):
        boxes_ids = tracker.update(detections)
        velocities = []

        for box_id in boxes_ids:
            x, y, w, h, id = box_id
            cv2.putText(cropped_frame, str(id), (x, y+30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            object_velocity = tracker.velocity(id, fps)
            velocities.append(object_velocity)
            filtered_velocities = [v for v in velocities if v is not None]
            if len(filtered_velocities) > 0:
                avg_velocity = sum(filtered_velocities) / len(filtered_velocities)
                avg_velocity = np.round(avg_velocity, 2)
                cv2.putText(cropped_frame, str(avg_velocity) + "cm/s", (x, y+150), cv2.FONT_HERSHEY_PLAIN, 2, (0, 100, 255), 2)

        return velocities


    """def main(self):
        #Kalmannfilter
        kf = KalmanFilter()
        #Tracker
        #Tracker erstellen
        tracker = Tracker()

        imageProcessor = ImageProcessor()
        object_ids = []

        cap = cv2.VideoCapture("F:/mariu/Desktop/Projekt 3 HS/Videos/AruCoGemischt.mp4")


        #Wartezeit für flüssiges Video berechnen
        fps = cap.get(cv2.CAP_PROP_FPS)
        wait_time = int(1000/fps)                    #Formel für Wartezeit
        print("FPS: ", fps)

        while True:
            ret, frame = cap.read()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # # Resize the frame
            resized_frame = imageProcessor.resize_frame(frame, 100)
            # # Detect the markers
            marker_centers = imageProcessor.detect_markers(resized_frame)
            # # Transform the frame to a bird's eye view
            warped_frame = imageProcessor.transform_to_birds_eye_view(resized_frame, marker_centers)
            # # Crop the frame
            cropped_frame = imageProcessor.crop_warped_image(warped_frame)

            # # Detect the objects
            detections, velocities = ObjectDetection.detect_objects(cropped_frame, kf)
            print(detections)
            print(velocities)
            # # Track the objects
            velocities = ObjectDetection.object_tracking(cropped_frame, tracker, detections, fps)
            #print(velocities)

# # Show the frame
            cv2.imshow("Frame", cropped_frame)
            cv2.waitKey(wait_time)

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ObjectDetection().main()"""

