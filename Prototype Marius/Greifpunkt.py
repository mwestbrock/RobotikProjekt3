import cv2
import numpy as np

def grippingPoint(bounding_box,drawCircle): #bounding_box = Pfad zum Bild bounding_box = Extrahiertes Objekt als Bild



    image = bounding_box
    #Graustufenbild


    #Euklidische Distanz von Binärem-Bild
    eukl = cv2.distanceTransform(image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    dist_output = cv2.normalize(eukl, None, 0, 1.0, cv2.NORM_MINMAX)

    #Greifpunkt bestimmen
    grippingPoint = np.where(dist_output == np.max(dist_output))

    #Greifpunkt in Bild zeichnen
    cv2.circle(drawCircle, (grippingPoint[1][0], grippingPoint[0][0]), 5, (0,0,255), -1)

    #Greifpunkt Koordinaten
    x = grippingPoint[0][0].item()
    y = grippingPoint[1][0].item()
    #cv2.imshow('Greifpunkt',dist_output)
    #cv2.waitKey(int(1000/30))


    return x, y






#grippingPoint('C:/Users/mariu/4. Semester/Projekt/Bilder/Einhorn.png')
