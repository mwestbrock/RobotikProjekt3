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
=======
#Bild einlesen
image = cv2.imread('C:/Users/mariu/4. Semester/Projekt/Bilder/Katze.png')

#Bild in Binär Konvertieren
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
ret, binary_image = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

#Euklidische Distanz von Binärem-Bild
transformed_image = cv2.distanceTransform(binary_image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
dist_output = cv2.normalize(transformed_image, None, 0, 1.0, cv2.NORM_MINMAX)

#Greifpunkt bestimmen
grippingPoint = np.where(dist_output == np.max(dist_output))

#Greifpunkt in Bild zeichnen
cv2.circle(image, (grippingPoint[1][0], grippingPoint[0][0]), 70, (0,0,255), -1)

#Greifpunkt Koordinaten
x = grippingPoint[0][0].item()
y = grippingPoint[1][0].item()

#Ausgabe Koordinaten
print("Greifpunkt Koordiante: ",x ,"|",y)

cv2.imshow('Greifpunkt',image)
cv2.waitKey(0)
cv2.destroyAllWindows()


>>>>>>> 8cf177c177e5eb87764c2fe824a38dcfc719df92
