import cv2
import numpy as np
from KalmanFilter import Greifpunkt as gp
from skimage.transform import resize
import joblib


# Funktion für die Objektunterscheidung
def predict_with_svm(model, bounding_box, textImage):

    Categories = ['cat_binary', 'unicorn_binary']
    img_resize = resize(bounding_box, (150, 150, 1))
    flattened_img = img_resize.flatten()
    l = [flattened_img]

    # Perform prediction on the input image
    #probability = model.predict_proba(l)
    #for ind, val in enumerate(Categories):
    #print(f'{val} = {probability[0][ind] * 100}%')
    #print("The predicted image is:", Categories[model.predict(l)[0]])
    cv2.putText(textImage, Categories[model.predict(l)[0]], (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return

#MAIN PROGRAMM
#Pfad svm Model
svm_path = 'F:/mariu/Desktop/ProjektBilder/pics/svm_model_binary.pkl'
#Pfad zum Video
video_path = 'F:/mariu/Desktop/Projekt 3 HS/Videos/CatsUnicorns.mp4'

# SVM-Modell laden
svm_model = joblib.load(svm_path)

# Video laden
cap = cv2.VideoCapture(video_path) #VideoCapture(0) für Kamera

# Überprüfen, ob das Video erfolgreich geladen wurde
if not cap.isOpened():
    print("Fehler beim Öffnen des Videos")
    exit()

#Wartezeit für flüssiges Video berechnen
fps = cap.get(cv2.CAP_PROP_FPS)
wait_time = int(1000/fps)                    #Formel für Wartezeit
print("FPS: ", fps)



# Video, Frame für Frame durchgehen und anzeigen
while True:
    ret,frame = cap.read()
    if not ret:
        print("Fehler beim Lesen des Frames")
        break
    #Geschwindigkkeit der Objekte berechnen

    #Bild skalieren
    scale_percent = 50 # Prozentsatz der ursprünglichen Größe
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)

    # Skaliere das Bild auf die neue Größe
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

    #Perspektive transformieren
    rows,cols,ch = frame.shape
    trFrameX = 900
    trFrameY = 180
    pts1 = np.float32([[89,232],[635,286],[75,382],[633,407]])
    pts2 = np.float32([[0,0],[trFrameX,0],[0,trFrameY],[trFrameX,trFrameY]])
    M = cv2.getPerspectiveTransform(pts1,pts2)

    transformedFrame = cv2.warpPerspective(frame,M,(trFrameX,trFrameY))

    #Förderband bereich auswählen
    yBeltW = trFrameY - 118
    yBeltL = trFrameY
    xBeltW = 0
    xBeltL = trFrameX

    conveyerBelt = transformedFrame[yBeltW:yBeltL, xBeltW:xBeltL]
    cv2.drawMarker(transformedFrame, (xBeltW, yBeltW), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)


    # Konvertiere das Bild in Graustufen
    gray = cv2.cvtColor(conveyerBelt, cv2.COLOR_BGR2GRAY)

    #Schwellenwert einstellen
    schwellenwert = 200
    ret, thresh = cv2.threshold(gray, schwellenwert, 255, cv2.THRESH_BINARY)

    #Kontur finden
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Grenzen für SVM
    xminSVM = 200
    yminSVM = 0
    xmaxSVM = 820
    ymaxSVM = trFrameY

    #Grenzen einzeichnen
    cv2.line(transformedFrame, (xminSVM, yminSVM), (xminSVM, ymaxSVM), (0, 0, 255), 1)
    cv2.line(transformedFrame, (xmaxSVM, yminSVM), (xmaxSVM, ymaxSVM), (0, 0, 255), 1)



    #Bounding-Box einzeichnen
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        #Boundingbox extrahieren
        bounding_box = thresh[y:y+h, x:x+w]


        if x > xminSVM  and x + w < xmaxSVM:

            #Bounding-Box einzeichnen
            cv2.rectangle(conveyerBelt, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #Bild für SVM vorbereiten
            draw_boundingbox = conveyerBelt[y:y+h, x:x+w]
            #Objektunterscheidung
            predict_with_svm(svm_model,bounding_box,draw_boundingbox)
            #Greifpunkt berechnen
            gp.grippingPoint(bounding_box,draw_boundingbox)


    cv2.imshow('Video', transformedFrame)
    if cv2.waitKey(wait_time) & 0xff == ord('q'): #q drücken um Video zu beenden
        break




# Video-Objekt freigeben
cap.release()
cv2.destroyAllWindows()


