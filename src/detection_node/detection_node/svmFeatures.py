import cv2
import pandas as pd
import joblib
import numpy as np
import sklearn

""" Class to extract features from a bounding_box/frame and classify it with a SVM """""

class SvmFeatures():

    def extract_number_of_corners(bounding_box):
        """ Extract number of corners from an object inside a bounding box/frame
            Args:
                thresh: thresholded frame
            Returns:
                number_of_corners: number of corners in the frame
        """
        corners = cv2.goodFeaturesToTrack(bounding_box, 100, 0.01, 10)
        number_of_corners = len(corners)
        #print("Number of corners: ", number_of_corners)
        #Draw corners
        #for corner in corners:
        #   x, y = corner.ravel()
        #   centercoords = (int(x), int(y))
        #   print("x: ", x, "y: ", y)
        #   cv2.circle(frame, centercoords, 3, 255, -1)
        return number_of_corners
    
    def extract_area(bounding_box):
        """ Extract area from an object inside a bounding box/frame
            Args:
                thresh: thresholded frame
            Returns:
                area: area of the frame
        """
        
        contours, hierarchy = cv2.findContours(bounding_box, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area = 0
        for cnt in contours:
            area += cv2.contourArea(cnt)
        #print("Area: ", area)
        return area

    def biggest_radius(bounding_box):
        """ Extract biggest radius from an object inside a bounding box/frame
            Args:
                thresh: thresholded frame
            Returns:
                radius: biggest radius of the frame
        """

        contours, hierarchy = cv2.findContours(bounding_box, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        radius = 0
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
        #draw circle around object
        #center = (int(x),int(y))
        #radius = int(radius)
        #cv2.circle(frame,center,radius,(0,255,0),2)
        #Bild anzeigen
        #cv2.imshow("Frame", frame)
        #cv2.waitKey(0)
        #print("Radius: ", radius)
        return radius
    
    def extract_features(bounding_box):
        """ Extract features from a frame
            Args:
                thresh: thresholded frame
            Returns:
                features: features of the frame [number_of_corners, area, radius]
        """
        number_of_corners = SvmFeatures.extract_number_of_corners(bounding_box)
        area = SvmFeatures.extract_area(bounding_box)
        radius = SvmFeatures.biggest_radius(bounding_box)
        #print(number_of_corners,area, radius, sep=",")
        return [number_of_corners, area, radius]
    
    def classify_frame(features):
        """ Classify frame with a SVM
            Args:
                features: features of the frame [number_of_corners, area, radius]
            Returns:
                svm_prediction: prediction of the SVM
        """
        svm_loaded =joblib.load('/home/mbird/RobotikProjekt3/ros2_workspace/src/detection_node/detection_node/SVM/svm_v4.pkl')
        features = np.array(features).reshape(1, -1)
        X = pd.DataFrame(features, columns=['number_of_corners', 'area', 'radius'])
        svm_prediction = svm_loaded.predict(X)
        return svm_prediction
