import cv2
import joblib
import numpy as np
import pandas as pd

from KalmanFilter.imageProcessorAruco import ImageProcessor

# Laden des gespeicherten Modells
svm_loaded = joblib.load('F:/mariu/Desktop/Projekt 3 HS/Trainingsdaten/svm_unicorn_cat.pkl')

def extract_number_of_corners(thresh):

    corners = cv2.goodFeaturesToTrack(thresh, 100, 0.01, 10)
    number_of_corners = len(corners)
    print("Number of corners: ", number_of_corners)
    #Draw corners
    #for corner in corners:
    #   x, y = corner.ravel()
    #   centercoords = (int(x), int(y))
    #   print("x: ", x, "y: ", y)
    #   cv2.circle(frame, centercoords, 3, 255, -1)
    #Bild anzeigen
    #cv2.imshow("Frame", frame)
    #cv2.waitKey(0)

    return number_of_corners

#Extract area of white object in frame
def extract_area(thresh):

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area = 0
    for cnt in contours:
        area += cv2.contourArea(cnt)
    print("Area: ", area)

    return area

#Extract Number of white Pixels in frame

def biggest_radius(thresh):

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    radius = 0
    for cnt in contours:
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        #draw circle around object
        #center = (int(x),int(y))
        #radius = int(radius)
        #cv2.circle(frame,center,radius,(0,255,0),2)
        #Bild anzeigen
        #cv2.imshow("Frame", frame)
        #cv2.waitKey(0)

    print("Radius: ", radius)
    return radius

# Definiere eine Funktion zum Klassifizieren eines Frames
def classify_frame(frame):
    # Führe hier die notwendige Vorverarbeitung auf dem Frame durch (z.B. Größenänderung, Feature-Extraktion)


    #Object Detection
    #converts to binary image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #Threshold
    threshold = 150
    ret, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 3000:
            x, y, w, h = cv2.boundingRect(cnt)
            if x > 0 and x +w < 40*10:

                cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                extracted_bb = thresh[y:y + h, x:x + w]
                #Bereich für Image saving
                roi = thresh[0:450, 0:40*5]
                #Bild anzeigen
                #cv2.imshow("Frame", roi)
                #cv2.waitKey(0)
                #Extrahiere features aus dem Bild
                number_of_corners = extract_number_of_corners(extracted_bb)
                area = extract_area(extracted_bb)
                radius = biggest_radius(extracted_bb)

                features = [number_of_corners, area, radius]
                features = np.array(features).reshape(1, -1)

                X = pd.DataFrame(features, columns=['number_of_corners', 'area', 'radius'])

                    # Klassifiziere den Frame mit der SVM
                svm_prediction = svm_loaded.predict(X)

                return svm_prediction

# Pfad zum Video
video_path = 'F:/mariu/Desktop/Projekt 3 HS/Videos/ArucoGemischt.mp4'

# Öffne das Video
video = cv2.VideoCapture(video_path)

# Loop über alle Frames im Video
while video.isOpened():
    # Lies den aktuellen Frame
    ret, frame = video.read()
    imageProcessor = ImageProcessor()

    resized_frame = imageProcessor.resize_frame(frame, 100)
    marker_centers = imageProcessor.detect_markers(resized_frame)
    warped_frame = imageProcessor.transform_to_birds_eye_view(resized_frame, marker_centers)
    cropped_frame = imageProcessor.crop_warped_image(warped_frame)

    if ret:
        # Klassifiziere den Frame
        prediction = classify_frame(cropped_frame)
        prediction = str(prediction)

        # Zeige das Ergebnis auf dem Frame an
        cv2.putText(cropped_frame, prediction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Frame', cropped_frame)

        # Warte auf eine Tastatureingabe
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Schließe das Video und zerstöre die Fenster
video.release()
cv2.destroyAllWindows()
