"""Node for object detection.
This node detects and tracks objects, sets the gripping point and calculates the velocity of the objects to be published.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from . import svmFeatures
import sklearn
from filterpy.kalman import KalmanFilter
import math
import time
import joblib
from tutorial_interfaces.msg import Detection
from . import tracker
from . import velocity
from . import gripPointEukl


objectVelocity = velocity.Velocity()
tracker = tracker.Tracker()

class DetectedObject:
    """Class for detected objects.
    Attributes:
        obj_id (int): ID of the object
        x (int): x coordinate of the gripping point
        y (int): y coordinate of the gripping point
        classification (int): classification of the object (Unicorn = 0, Cat = 1)
        velocity (float): velocity of the object
    """
    def __init__(self, obj_id, x, y, classification, velocity):
        self.obj_id = obj_id
        self.x = x
        self.y = y
        self.classification = classification  # Unicron = 0, Cat = 1
        self.velocity = velocity


class DetectionNode(Node):
    """Node for object detection.
    Attributes:
        threshold (int): Threshold fo object detection
        minItemArea (int): Minimum area of an object to be detected
        minDetectionBorder (int): Start detection at  0cm
        maxDetectionBorder (int): Stop detection at 11cm
        publishBorder (int): Publish Objects at defined border (8cm)
        id (int): ID of the object
        center_points (dict): Center points of the objects to be tracked
        detectedObjects (dict): List of detected objects to be published
        starttime (list): Starttime of the objects to calculate the velocity
    """

    def __init__(self):
        self.threshold = 150            
        self.minItemArea = 3000         
        self.minDetectionBorder = 0     
        self.maxDetectionBorder = 440   
        self.publishBorder = 320             
        self.id = 0                     
        self.center_points = {}         
        self.detectedObjects = {}       
        self.starttime = [None, None, None, None, None, None, None, None, None, None] 

        
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            'cropped_image',
            self.image_callback,
            10
        )

        self.detection_publisher = self.create_publisher(
            Detection,
            'detection',
            10
        )

        self.timer = self.create_timer(1.0 / 30, self.publish_object)

    def publish_detected_objects(self, detectedObject):
        """Publishes the detected object.
        Args:
            detectedObject (DetectedObject): Detected object to be published
            
        """
        msg = Detection()
        msg.id = int(detectedObject.obj_id)
        msg.gpx = float(detectedObject.x)
        msg.gpy = float(detectedObject.y)
        msg.classification = int(detectedObject.classification)
        msg.velocity = float(detectedObject.velocity)
        self.detection_publisher.publish(msg)

    def image_callback(self, msg):
        """Callback function for the image subscriber.
        Args:
            msg (Image): Image message from the camera
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            processed_image = self.process_frame(cv_image, 30)
            cv2.imshow("Processed Image", processed_image)
            cv2.waitKey(1)

        except Exception as e:
            #print(e)
            pass

    def pixel_to_cm(self, pixel):
        """Converts pixel to cm.
        Args:
            pixel (int): Pixel to be converted
        Returns:
                int: Pixel converted to cm
        """
        return pixel / 40
    
    def cmToPixel(self, cm):
        """Converts cm to pixel.
        Args:
            cm (int): cm to be converted
        Returns:
                int: cm converted to pixel
        """
        return cm * 40
    
    def process_frame(self, cropped_frame, fps):
        """Processes the frame.
        Args:
            cropped_frame (Image): Image of the cropped frame
            fps (int): Frames per second
        Returns:
                Image: Processed image
        """
        # Convert to grayscale
        gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        # Apply threshold to convert to binary image
        ret, thresh = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)
        # Find contours in the binary image
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize list of detections for tracker
        detections = [] 
        # Iterate over all contours
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Check if objects are big enough to be detected
            if area > self.minItemArea:
                x, y, w, h = cv2.boundingRect(cnt)
                # Check if objects are in the detection range
                if self.minDetectionBorder < x < self.maxDetectionBorder - w:
                    
                    #cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Set center point of the object for velocity calculation
                    center_x = int(x + w / 2)

                    # Calculate velocity
                    velo = objectVelocity.objectVelocity(center_x, 4.5, 7, 0)

                    bounding_box = thresh[y:y + h, x:x + w]

                    detections.append([x, y, w, h])

                    # Machine Learning
                    # Extract features from bounding box
                    features = svmFeatures.SvmFeatures.extract_features(bounding_box)
                    # Predict class of object
                    prediction = svmFeatures.SvmFeatures.classify_frame(features)
                    # convert the prediction to an int
                    classification = int(prediction)

                    # Find gripping point
                    gpx, gpy = gripPointEukl.GripPoint.grippingPoint(bounding_box)
                    cx = x + gpx
                    cy = y + gpy
        # Update tracker with detections
        boxes_ids = tracker.update(detections)
        # Iterate over all detected objects
        for box_id in boxes_ids:
            x, y, w, h, self.id = box_id 
            #Add new object to the list of detected objects if it is not already in the list , else update the object
            if self.id not in self.center_points:
                self.center_points[self.id] = [(x, y)]
                self.detectedObjects[self.id]= DetectedObject(self.id, cx, cy, classification, objectVelocity.getAverageVelocity())
            # Check if the object is the same as the last one and update it
            else:
                self.center_points[self.id].append((x, y))
                for obj in self.detectedObjects:
                    if obj.obj_id == self.id:
                        obj.x = cx
                        obj.y = cy
                        obj.classification = classification
                        obj.velocity = objectVelocity.getAverageVelocity()
                        break

        return cropped_frame


   
    def publish_object(self):
        """Publishes the detected objects.
            Args:
                detectedObject (DetectedObject): Detected object to be published
        """
        for obj_id, obj in self.detectedObjects.items():  
            if obj.x > self.publishBorder:
                self.publish_detected_objects(obj) 
                time.sleep(3) 
                #self.get_logger().info("Object published")
        self.detectedObjects.clear()
        self.center_points.clear()



def main(args=None):
    """Main function.
    Args:
        args (list): List of arguments
    """
    rclpy.init(args=args)
    detection_node = DetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()