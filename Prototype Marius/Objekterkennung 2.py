import cv2
import numpy as np
import Förderbanderkennung


cap = cv2.VideoCapture(0)
while True:
    ret, img = cap.read()
    # Definieren Sie die Koordinaten der 4 Ecken des zu schneidenden Bereichs abgeleitet aus Förderbanderkennung
    pts = np.array(Förderbanderkennung.bereich(), np.int32)

    # Erstellen Sie eine leere Maske mit demselben Shape wie das ursprüngliche Bild
    mask = np.zeros_like(img)

    # Erstellen Sie eine Maske, die das Polygon enthält
    cv2.fillPoly(mask, [pts], (255, 255, 255))

    # Wenden Sie die Maske auf das ursprüngliche Bild an, um den Bereich auszuschneiden
    masked_image = cv2.bitwise_and(img, mask)
    #################################################

    # Konvertieren Sie das Bild in den HSV-Farbraum
    hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)

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
    cv2.drawContours(masked_image, contours, -1, (0, 0, 255), 2)

    # Zeigen Sie das zugeschnittene Bild an
    cv2.imshow("Zugeschnittenes Bild", masked_image)



    max_length = 0
    for contour in contours:
        length = cv2.arcLength(contour, True)
        if length > max_length:
            max_length = length


        if max_length > 169.5:
            print("Katze erkannt!")
        elif max_length <= 0:
            print("Kein Objekt erkannt")
        else:
             print("Einhorn erkannt")

    if max_length <= 0:
        print("Kein Objekt erkannt")


    #print(max_length)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
