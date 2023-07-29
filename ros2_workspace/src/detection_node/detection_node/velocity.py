import time
import cv2

class Velocity:
    """Class for calculating the velocity of an object.
    Attributes:
        starttimes (list): Starttime of the objects to calculate the velocity
        averageVelocity (list): List of average velocities of the objects
    """
    def __init__(self):
        """Initialize the attributes of the Velocity class."""
        self.starttimes = [None, None, None, None, None, None, None, None, None, None]
        self.averageVelocity = []

    def pixelToCm(self, pixel):
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
    
    def objectVelocity(self, center_x, startPoint, endPoint, idx):
        """Calculates the velocity of an object.
        Args:
            center_x (int): Center x coordinate of the object
            startPoint (int): Start point to begin the time measurement
            endPoint (int): End point to end the time measurement
            idx (int): Index of the object
        Returns:
                float: Velocity of the object
        """

        if center_x >= self.cmToPixel(startPoint) and self.starttimes[idx] is None:
            self.starttimes[idx] = time.monotonic_ns() 

        if center_x >= self.cmToPixel(endPoint) and self.starttimes[idx] is not None:
            endtime = time.monotonic_ns()
            duration = endtime - self.starttimes[idx]
            duration = duration / 1000000000
            self.starttimes[idx] = None

            if duration != 0:
                velocity = (endPoint - startPoint) / duration
                if velocity < 10:
                    self.averageVelocity.append(velocity)
                    return velocity 
                
    def getAverageVelocity(self):
        avgVelocity = sum(self.averageVelocity) / len(self.averageVelocity)
        return avgVelocity