import cv2
import numpy as np
import cv2.aruco as aruco
from imageProcessorAruco import ImageProcessor
import os


class SampleData:
    def main(self):
        imageProcessor = ImageProcessor()
        object_ids = []

        cap = cv2.VideoCapture("F:/mariu/Desktop/Projekt 3 HS/Videos/ArucoKatze.mp4")
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

            #Object Detection
            #converts to binary image
            gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

            ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 3000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    if x > 0 and x +w < 40*11:

                        cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        extracted_bb = thresh[y:y + h, x:x + w]
                        #Bereich für Image saving
                        roi = thresh[0:450, 0:450]
                        cv2.imshow("ROI", roi)

                        image_filename = f"Katze_{x}_{y}_{w}_{h}.jpg"
                        save_file_path = os.path.join("F:/mariu/Desktop/Projekt 3 HS/Trainingsdaten/Katzen", image_filename)
                        cv2.imwrite(save_file_path, roi)
                        print(f"Saved {image_filename}")








            cv2.imshow("Frame", cropped_frame)
            cv2.waitKey(wait_time)

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    SampleData().main()