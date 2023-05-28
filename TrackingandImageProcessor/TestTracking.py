#import everything from tracker
from Tracking.tracker import *

tracker = EuclideanDistTracker()

# Video laden
cap = cv2.VideoCapture('F:/mariu/Desktop/Projekt 3 HS/Videos/CatsUnicorns.mp4 ') #VideoCapture(0) für Kamera

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

    #Bild skalieren
    scale_percent = 50 # Prozentsatz der ursprünglichen Größe
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)

    # Skaliere das Bild auf die neue Größe
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

    #Perspektive transformieren
    rows,cols,ch = frame.shape
    pts1 = np.float32([[89,232],[635,286],[75,382],[633,407]])
    pts2 = np.float32([[0,0],[900,0],[0,200],[900,200]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(frame,M,(900,210))

    crop_img = dst[69:199, 0:900]
    cv2.drawMarker(dst, (0, 69), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

    # Konvertiere das Bild in Graustufen
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    #Schwellenwert einstellen
    schwellenwert = 200
    expand = 1
    ret, thresh = cv2.threshold(gray, schwellenwert, 255, cv2.THRESH_BINARY)

    #Kontur finden
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []

    #Bounding-Box einzeichnen
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(crop_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        bounding_box = crop_img[y:y + h, x:x + w]

        detections.append([x,y,w,h])

    #Object TRACKING
    boxes_ids = tracker.update(detections)
    for box_id in boxes_ids:
        x, y, w, h, id = box_id
        cv2.putText(crop_img, str(id), (x, y+30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.rectangle(crop_img, (x, y), (x + w, y + h), (0, 255, 0), 2)









    cv2.imshow('Video', dst)
    if cv2.waitKey(wait_time) & 0xff == ord('q'): #q drücken um Video zu beenden
        break








# Video-Objekt freigeben
cap.release()
cv2.destroyAllWindows()