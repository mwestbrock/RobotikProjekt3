import cv2

# Lade das Referenzbild
reference_image = cv2.imread('C:/Users/mariu/4. Semester/Projekt/Bilder/Katze_1.jpg')

# Erstelle einen Kamera-Objekt
camera = cv2.VideoCapture(0)

# Erstelle ein SIFT-Feature-Objekt
sift = cv2.SIFT_create()

while True:
    # Nehme ein Bild von der Kamera auf
    ret, frame = camera.read()

    # Führe die Objekterkennung durch
    kp1, des1 = sift.detectAndCompute(reference_image, None)
    kp2, des2 = sift.detectAndCompute(frame, None)

    # Führe das Feature-Matching durch
    matcher = cv2.FlannBasedMatcher()
    matches = matcher.knnMatch(des1, des2, k=2)

    # Filtere gute Matches
    good_matches = []
    for m, n in matches:
        if m.distance < 0.6 * n.distance:
            good_matches.append(m)

    # Zeichne die Übereinstimmungen auf das Bild
    output = cv2.drawMatches(reference_image, kp1, frame, kp2, good_matches, None)

    # Zeige das Ergebnisbild an
    cv2.imshow('Object Detection', output)

    # Breche die Schleife ab, wenn die 'q'-Taste gedrückt wird
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Beende die Kamera-Aufnahme
camera.release()

# Schließe alle offenen Fenster
cv2.destroyAllWindows()
