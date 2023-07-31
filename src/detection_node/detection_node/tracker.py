import math
import numpy as np
import cv2

class Tracker:
    """Class for tracking objects.
    Attributes:
        center_points (dict): Center points of the objects to be tracked
        id_count (int): ID of the object
        sameItemRange (int): Range in which an object is considered the same
    """
    def __init__(self):
        """Initialize the attributes of the Tracker class."""
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0
        self.sameItemRange = 50


    def update(self, objects_rect):
        """Update the tracker.
        Args:
            objects_rect (list): List of bounding boxes of the objects
        Returns:
            list: List of bounding boxes of the objects with IDs
        """
        objects_bbs_ids = []
        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            # Find out if that object was detected already
            same_object_detected = False
            for id, points in self.center_points.items():
                last_point = points[-1] if points else None
                if last_point is not None:
                    last_cx, last_cy = last_point
                    dist = math.hypot(cx - last_cx, cy - last_cy)
                    # If the distance between the center points is less than 50 pixels than it is the same object
                    if dist < self.sameItemRange:
                        points.append((cx, cy))
                        objects_bbs_ids.append([x, y, w, h, id])
                        same_object_detected = True
                        break
            # New object is detected we assign the ID to that object
            if not same_object_detected:
                self.center_points[self.id_count] = [(cx, cy)]
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1
        # If there are no objects detected, remove all the items from the dictionary
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center
        # Update the dictionary
        self.center_points = new_center_points.copy()
        return objects_bbs_ids


