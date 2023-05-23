import cv2
import numpy as np

def grippingPoint(bounding_box,drawCircle): #bounding_box = Pfad zum Bild bounding_box = Extrahiertes Objekt als Bild

    image = bounding_box

    #Euklidische Distanz von Binärem-Bild
    transformed_image = cv2.distanceTransform(image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    dist_output = cv2.normalize(transformed_image, None, 0, 1.0, cv2.NORM_MINMAX)

    #Greifpunkt bestimmen
    grippingPoint = np.where(dist_output == np.max(dist_output))

    #Greifpunkt in Bild zeichnen
    cv2.circle(drawCircle, (grippingPoint[1][0], grippingPoint[0][0]), 5, (0,0,255), -1)

    #Greifpunkt Koordinaten
    x = grippingPoint[0][0].item()
    y = grippingPoint[1][0].item()

    return x, y

    #Ausgabe Koordinaten
    #print("Greifpunkt Koordiante: ",x ,"|",y)

    #cv2.imshow('Greifpunkt',image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


#grippingPoint('C:/Users/mariu/4. Semester/Projekt/Bilder/Einhorn.png')
