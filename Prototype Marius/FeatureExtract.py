#Extract features from pictures and save them in a csv file

import os
import cv2
import numpy as np
#

#Number of corners of White Object in frame
def extract_number_of_corners(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    corners = cv2.goodFeaturesToTrack(thresh, 100, 0.01, 10)


    number_of_corners = len(corners)
    print("Number of corners: ", number_of_corners)
    #Draw corners
    #for corner in corners:
    #   x, y = corner.ravel()
    #   centercoords = (int(x), int(y))
    #   print("x: ", x, "y: ", y)
    #   cv2.circle(frame, centercoords, 3, 255, -1)
    #Bild anzeigen
    #cv2.imshow("Frame", frame)
    #cv2.waitKey(0)
    return number_of_corners

#Extract area of white object in frame
def extract_area(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area = 0
    for cnt in contours:
        area += cv2.contourArea(cnt)
    print("Area: ", area)
    return area
#Extract Number of white Pixels in frame

def biggest_radius(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

    print("Radius: ", radius)
    return radius
#Main

#Path to folder with pictures
path = "F:/mariu/Desktop/Projekt 3 HS/Trainingsdaten/Einhorn"
#Path to csv file
csv_path = "F:/mariu/Desktop/Projekt 3 HS/Trainingsdaten/Einhorn.csv"
#Open csv file
csv_file = open(csv_path, "w")
#Write header in csv file (name of features)
csv_file.write("number_of_corners, area, radius\n")
#Iterate through all pictures in folder
for filename in os.listdir(path):
    #Open picture
    img = cv2.imread(os.path.join(path, filename))
    #Extract features
    number_of_corners = extract_number_of_corners(img)
    area = extract_area(img)
    biggest_radius_around_object = biggest_radius(img)
    #Write features in csv file
    csv_file.write(f"{number_of_corners}, {area}, {biggest_radius_around_object}\n")
#Close csv file
csv_file.close()

#Mai Method to test functios
"""def main():
    img = cv2.imread("F:/mariu/Desktop/Projekt 3 HS/Trainingsdaten/Einhorn/Einhorn_242_36_193_167.jpg")
    extract_number_of_corners(img)
    extract_area(img)
    biggest_radius_around_object(img)

if __name__ == "__main__":
    main()"""
