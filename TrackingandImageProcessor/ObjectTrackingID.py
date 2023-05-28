from tracker import *
import queue
from imageProcessor import *

#MAIN PROGRAMM

#Pfad zum Video
video_path = 'F:/mariu/Desktop/Projekt 3 HS/Videos/CatsUnicorns.mp4'
#Tracker erstellen
tracker = Tracker()
#ID QUEUE
id_queue = queue.Queue()
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
#Bild Skalierung und Verzerrung
processor = ImageProcessor()


# Video, Frame für Frame durchgehen und anzeigen
while True:
    ret,frame = cap.read()
    if not ret:
        print("Fehler beim Lesen des Frames")
        break
    #Bild Transformation und Objekterkennungs Bereich
    transformed_frame, detection_area  = processor.process_frame(frame)


    # Konvertiere das Bild in Graustufen
    gray = cv2.cvtColor(detection_area, cv2.COLOR_BGR2GRAY)

    #Schwellenwert einstellen
    schwellenwert = 200
    ret, thresh = cv2.threshold(gray, schwellenwert, 255, cv2.THRESH_BINARY)

    #Kontur finden
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    velocities = []

    #Bereich für OBJECT DETECTION

    #Bounding-Box einzeichnen
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 3000 :
            x, y, w, h = cv2.boundingRect(cnt)

            if x > processor.getxMinDet() and x + w < processor.getxMaxDet():
                #Boundingbox extrahieren
                bounding_box = thresh[y:y+h, x:x+w]
                detections.append([x,y,w,h])


                #Object TRACKING
    boxes_ids = tracker.update(detections)
    for box_id in boxes_ids:
        x, y, w, h, id = box_id

        #Objekt ID einzeichnen
        cv2.putText(detection_area, str(id), (x, y+30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        #Boundingbox einzeichnen
        cv2.rectangle(detection_area, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #Geschwidigkeit von jedem Objekt berechnen
        object_velocity = tracker.velocity(id, fps)
        #Average Velocity
        velocities.append(object_velocity)
        filtered_velocities = [v for v in velocities if v is not None]
        if len(filtered_velocities) > 0:
            avg_velocity = sum(filtered_velocities) / len(filtered_velocities)
            avg_velocity = np.round(avg_velocity, 2)
            #Geschwindigkeit einzeichnen
            cv2.putText(detection_area, str(avg_velocity)+"cm/s", (x, y+60), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)





    cv2.imshow('Video', transformed_frame)
    if cv2.waitKey(wait_time) & 0xff == ord('q'): #q drücken um Video zu beenden
        break




# Video-Objekt freigeben
cap.release()
cv2.destroyAllWindows()


