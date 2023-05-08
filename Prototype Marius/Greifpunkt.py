import cv2
import numpy as np

# Bild laden
img = cv2.imread('F:/mariu/Desktop/Projekt 3 HS/Einhorn.png')

# Graustufenbild erzeugen
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Kantenerkennung mit Canny durchführen
edges = cv2.Canny(gray, 100, 200)

# Konturen finden
contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#

# Leeres Bild für Kontur erstellen
contour_img = np.zeros_like(img)

# Kontur in das Bild zeichnen
cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)

# Konturbild anzeigen
cv2.imshow('Konturbild', contour_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
