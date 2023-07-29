import cv2
import numpy as np

class GripPoint():
    def grippingPoint(bounding_box):
        """A function to calculate the best gripping point of an object in the bounding box, using the euclidean distance transform.
            Returns the coordinates of the gripping point of the object in the bounding box.
            Args:
                bounding_box (Image): Image of the bounding box

            Returns:   
                    int: x coordinate of the gripping point
                    int: y coordinate of the gripping point
        """
        image = bounding_box
        _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
        eukl = cv2.distanceTransform(binary_image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        dist_output = cv2.normalize(eukl, None, 0, 1.0, cv2.NORM_MINMAX)
        grippingPoint = np.unravel_index(np.argmax(dist_output), dist_output.shape)
        x = grippingPoint[1]
        y = grippingPoint[0]
        #cv2.circle(bounding_box, (x, y), 5, (0, 0, 0), -1) # Draw a circle around the gripping point
        return x, y