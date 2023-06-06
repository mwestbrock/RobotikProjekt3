import cv2
import numpy as np

class ImageProcessor:
    def __init__(self, scale_percent=50, trFrameX=900, trFrameY=180):
        self.scale_percent = scale_percent
        self.trFrameX = trFrameX
        self.trFrameY = trFrameY
        self.pts1 = np.float32([[90, 232], [639, 286], [76, 390], [634, 415]])
        self.pts2 = np.float32([[0, 0], [trFrameX, 0], [0, trFrameY], [trFrameX, trFrameY]])
        self.M = cv2.getPerspectiveTransform(self.pts1, self.pts2)

        self.yBeltW = trFrameY - 135
        self.yBeltL = trFrameY
        self.xBeltW = 0
        self.xBeltL = trFrameX

        self.xminDet = 200
        self.yminDet = 0
        self.xmaxDet = 814
        self.ymaxDet = trFrameY


    def process_frame(self, frame):
        # Bild skalieren
        width = int(frame.shape[1] * self.scale_percent / 100)
        height = int(frame.shape[0] * self.scale_percent / 100)
        dim = (width, height)
        scaled_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

        # Perspektive transformieren
        transformed_frame = cv2.warpPerspective(scaled_frame, self.M, (self.trFrameX, self.trFrameY))

        # Förderbandbereich auswählen

        #Nullpunkt einzeichnen
        cv2.drawMarker(transformed_frame, (self.xBeltW, self.yBeltW), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        #cv2.line(transformed_frame, (self.xminDet, self.yminDet), (self.xminDet, self.ymaxDet), (0, 0, 255), 1)
        #cv2.line(transformed_frame, (self.xmaxDet, self.yminDet), (self.xmaxDet, self.ymaxDet), (0, 0, 255), 1)

        detection_area = transformed_frame[self.yBeltW:self.yBeltL, self.xBeltW:self.xBeltL]

        cv2.drawMarker(detection_area, (0,0), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        return  transformed_frame, detection_area

    def getxMinDet(self):
        return self.xminDet

    def getxMaxDet(self):
        return self.xmaxDet

    def getyMinDet(self):
        return self.yminDet

    def getyMaxDet(self):
        return self.ymaxDet


