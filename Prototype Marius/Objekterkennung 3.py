import cv2
import numpy as np
import Förderbanderkennung

# Öffnen der Kamera
cap = cv2.VideoCapture(0)

# Definieren des Bereichs, den wir anzeigen möchten
points = np.array(Förderbanderkennung.bereich(), dtype=np.float64)
roi_mask = np.zeros((480, 640), dtype=np.uint8)
cv2.fillPoly(roi_mask, [points.astype(np.int32)], 255)

while True:
    # Lesen eines Frames aus der Kamera
    ret, frame = cap.read()

    # Anwenden der Maske auf den Frame
    roi = cv2.bitwise_and(frame, frame, mask=roi_mask)

    # Vergrößern des Bildausschnitts
    resize = 1
    zoomed_roi = cv2.resize(roi, None, fx=resize, fy=resize, interpolation=cv2.INTER_LANCZOS4)

    # Konvertieren Sie das Bild in den HSV-Farbraum
    hsv = cv2.cvtColor(zoomed_roi, cv2.COLOR_BGR2HSV)

    # Definieren Sie den Bereich der grünen Farbe in HSV
    lower_green = np.array([60, 10, 50])
    upper_green = np.array([255, 255, 254])

    # Definieren Sie den Bereich der weißen Farbe in HSV
    lower_white = np.array([0, 1, 0])
    upper_white = np.array([255, 10, 255])

    # Erstellen Sie eine Maske, die nur die grüne Farbe enthält
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # Erstellen Sie eine Maske, die nur die weiße Farbe enthält
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    # Führen Sie eine Bitweise UND-Operation durch, um alle weißen Objekte auf einem grünen Hintergrund zu erhalten
    result_mask = cv2.bitwise_and(green_mask, white_mask)

    # Extrahieren Sie die Konturen der weißen Objekte
    contours, hierarchy = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Zeichnen Sie die Konturen der gefundenen Objekte auf das Bild
    cv2.drawContours(zoomed_roi, contours, -1, (0, 0, 255), 2)

    # Zeigen Sie das zugeschnittene Bild an
    cv2.imshow("Zugeschnittenes Bild", zoomed_roi)

    max_length = 0.0
    for contour in contours:
        length = cv2.arcLength(contour, True)
        if length > max_length:
            max_length = length

        if max_length > 169.5 and max_length < 175.0:
            print("Katze erkannt!")
        elif max_length <= 0:
            print("Kein Objekt erkannt")
        elif max_length < 169.5 and max_length > 165.0:
            print("Einhorn erkannt")

    if max_length <= 0:
        print("Kein Objekt erkannt")



    # Beenden der Schleife bei Betätigung der Taste 'q'
    if cv2.waitKey(1) == ord('q'):
        break

# Freigeben der Ressourcen
cap.release()
cv2.destroyAllWindows()
