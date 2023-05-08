import cv2
import numpy as np

# Laden des Bildes
cap = cv2.VideoCapture(0)
#Erstes Frame auslesen
ret, frame = cap.read()
#Video Feed beenden
cap.release()

img = cv2.imread('C:/Users/mariu/4. Semester/Projekt/Bilder/Einhorn_1.jpg') #frame

# Konvertieren des Bildes in den HSV-Farbraum
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Definieren des Farbbereichs, der erkannt werden soll
lower_green = np.array([60, 40, 50])
upper_green = np.array([85, 255, 100])

# Erstellen einer Maske, um nur Pixel im definierten Farbbereich zu behalten
mask = cv2.inRange(hsv, lower_green, upper_green)

# Finden der Konturen um die grünen Bereiche im Bild
contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# größte Fläche
max_area = 0
best_contour = None
for contour in contours:
    area = cv2.contourArea(contour)
    if area > max_area:
        max_area = area
        best_contour = contour



# Zeichnen der Kontur mit der größten Fläche
if best_contour is not None:
    rect = cv2.minAreaRect(best_contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img, [box], 0, (0, 255, 0), 2)


# Anzeigen des Ergebnisbildes
def bereich():
    bereich = cv2.boxPoints(rect)
    return bereich

print(bereich())

cv2.imshow('Förderbanderkennung', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
