import cv2
import numpy as np
import time

#Pfad zum Video
video_path = 'F:/mariu/Desktop/Projekt 3 HS/Videos/CatsUnicorns.mp4'

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

#Grenzen für Zeitmessung
start_position =117
end_position = 289
#Umrechnungsfaktor Pixel in cm
pixel_to_cm = 8/172

#Liste für Geschwindigkeiten
velocities = []
#Startzeit
start_time = None



# Video, Frame für Frame durchgehen und anzeigen
while True:
    ret,frame = cap.read()
    if not ret:
        print("Fehler beim Lesen des Frames")
        break

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
    trFrameY = 200
    pts1 = np.float32([[89,232],[635,286],[75,382],[633,407]])
    pts2 = np.float32([[0,0],[trFrameX,0],[0,trFrameY],[trFrameX,trFrameY]])
    M = cv2.getPerspectiveTransform(pts1,pts2)

    transformedFrame = cv2.warpPerspective(frame,M,(trFrameX,trFrameY))

    #Förderband bereich auswählen
    yBeltW = trFrameY - 118
    yBeltL = trFrameY - 20
    xBeltW = 0
    xBeltL = trFrameX

    conveyerBelt = transformedFrame[yBeltW:yBeltL, xBeltW:xBeltL]
    cv2.drawMarker(transformedFrame, (xBeltW, yBeltW), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

    #grenzen einzeichnen
    #cv2.line(transformedFrame, (start_position, 0), (start_position, yBeltL), (0, 255, 0), 2)
    #cv2.line(transformedFrame, (end_position, 0), (end_position, yBeltL), (0, 255, 0), 2)

    #Bild in Graustufen konvertieren
    gray = cv2.cvtColor(conveyerBelt, cv2.COLOR_BGR2GRAY)

    schwellenwert = 200
    #Bild in Binärbild umwandeln
    ret, thresh = cv2.threshold(gray, schwellenwert, 255, cv2.THRESH_BINARY)

    pixel_value_start = thresh[45,start_position]
    pixel_value_end = thresh[45,end_position]
    #print(pixel_value_start)
    #print(pixel_value_end)

    if pixel_value_start > 250 and start_time == None:
        start_time = time.time()
    if pixel_value_end > 250 and start_time != None:
        end_time = time.time()
        #Delta time
        delta_time = end_time - start_time
        #Geschwindigkeit berechnen unter berücksichtigung der Frames
        distance = end_position - start_position
        velocity = distance / delta_time * pixel_to_cm
        velocities.append(velocity)
        #Durchschnittsgeschwindigkeit berechnen
        average_velocity = sum(velocities) / len(velocities)
        print("Durchschnittsgeschwindigkeit: ",round(average_velocity,3),"cm/s")
        #Zeit zurücksetzen
        start_time = None

    cv2.imshow('Video', transformedFrame)
    if cv2.waitKey(wait_time) & 0xff == ord('q'): #q drücken um Video zu beenden
        break

# Video-Objekt freigeben
cap.release()
cv2.destroyAllWindows()


