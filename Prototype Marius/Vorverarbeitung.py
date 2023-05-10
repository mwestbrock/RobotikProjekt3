import cv2
import numpy as np

# Bild laden und in Graustufenbild konvertieren
img = cv2.imread('F:/mariu/Desktop/Projekt 3 HS/Bilder/Scale.jpg')

# Definiere die neue Größe des Bildes
scale_percent = 50 # Prozentsatz der ursprünglichen Größe
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)

# Skaliere das Bild auf die neue Größe
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Wenden Sie einen Canny-Filter an, um Kanten zu erkennen
edges = cv2.Canny(gray, 50, 150,apertureSize=3)
lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=255)

# Zeichnen Sie die Linien auf dem Bild
"""for line in lines:
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

"""
# Schwellenwert für Schwarz setzen
threshold = 100

# Quadrate im Bild finden
squares = cv2.findContours((gray < threshold).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
squares = squares[0] if len(squares) == 2 else squares[1]

# Bounding-Boxen mit bestimmtem Seitenverhältnis und Flächeninhalt entfernen
tolerance = 0.5  # einstellbare Toleranz für das Seitenverhältnis
min_area = 350  # Mindestfläche für Bounding-Boxen
max_area = 490  # Maximalfläche für Bounding-Boxen
new_squares = []
for contour in squares:
    # Umgebendes Rechteck um das Quadrat finden
    x, y, w, h = cv2.boundingRect(contour)

    # Seitenlängen des Quadrats bestimmen
    side1 = min(w, h)
    side2 = max(w, h)

    # Seitenverhältnis des Quadrats bestimmen
    ratio = float(side1) / float(side2)

    # Wenn das Verhältnis nicht dem eines Quadrats entspricht (innerhalb der Toleranz), das Quadrat überspringen
    if not (1 - tolerance <= ratio <= 1 + tolerance):
        continue

    # Fläche der Bounding-Box berechnen
    area = w * h

    # Wenn die Fläche zu klein oder zu groß ist, die Bounding-Box überspringen
    if area < min_area or area > max_area:
        continue

    # Bounding-Box zum neuen Array hinzufügen
    new_squares.append(contour)

marker_coords = []
marker_coords_2 = []
y_length = 100
# Schleife über die Bounding-Boxen
for contour in new_squares:
    x, y, w, h = cv2.boundingRect(contour)

    # Finden Sie die konvexe Hülle der Kontur
    #hull = cv2.convexHull(contour)
    #print(hull)
    # Loop durch die Eckpunkte der konvexen Hülle und zeichnen Sie sie auf dem Bild
    #for point in hull:
    #    cv2.drawMarker(img, tuple(point[0]), (0, 255, 0))

    #cv2.drawContours(img, [hull], 0, (0, 255, 0), 2, lineType=cv2.LINE_AA)


    y_length = y_length+2.5
    #Marker-Koordinaten hinzufügen
    marker_coords.append((x, y))
    marker_coords_2.append((x,y+y_length))

    # Marker zeichnen
    cv2.drawMarker(img, (x, y), (0, 0, 255))



# Polylinie zeichnen
#cv2.polylines(img, [np.array(marker_coords)], False, (0, 0, 255), 1)
#cv2.drawContours(img,new_squares,-1,(0,255,0),1)



#################################
src_pts = np.float32([marker_coords[-1], marker_coords[1], marker_coords_2[1], marker_coords_2[-1]])

dst_pts = np.float32([[0, 0], [img.shape[1], 0], [img.shape[1], img.shape[0]], [0, img.shape[0]]])

# Transformationsmatrix berechnen
M = cv2.getPerspectiveTransform(src_pts, dst_pts)

# Bild in Vogelperspektive transformieren
warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

y_line = 100
cv2.line(warped,(0,y_line),(1000,y_line),(255,0,0),1)


# Bild anzeigen
cv2.imshow('image', img)
#cv2.imshow('warped', warped)
cv2.waitKey(0)
cv2.destroyAllWindows()
