import cv2
import numpy as np

# Lade das Bild
img = cv2.imread('C:/Users/mariu/4. Semester/Projekt/Bilder/Katze_1.jpg')

# Wähle einen Bildausschnitt aus
#x, y, w, h = cv2.selectROI(img)

# Beschränke das Bild auf den ausgewählten Ausschnitt
img = img[239:239+68, 247:247+151]

# Konvertiere das Bild in Graustufen
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Wende den Schwellenwert an, um nur weiße Objekte zu bekommen
ret, thresh = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)

# Finde die Konturen der weißen Objekte
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Zeichne eine Bounding-Box um jedes weiße Objekt
for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

# Zeige das Ergebnis
cv2.imshow('Ergebnis', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
