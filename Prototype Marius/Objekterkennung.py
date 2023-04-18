import cv2

# Bild auslesen
img = cv2.imread('C:/Users/mariu/4. Semester/Projekt/Bilder/Katze_1.jpg')   #cv2.VideoCapture(0)

# Ausschnitt auswählen
#x, y, w, h = cv2.selectROI(img)
#print(x)
#print(y)
#print(w)
#print(h)

# Bild ausschnitt beschränken
img = img[239:239+68, 247:247+151]

# Konvertiere das Bild in Graustufen
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#Schwellenwert einstellen
schwellenwert = 130
ret, thresh = cv2.threshold(gray, schwellenwert, 255, cv2.THRESH_BINARY)

#Kontur finden
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#Bounding-Box einzeichnen
for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

#Bild anzeigen
cv2.imshow('Objekterkennung', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
