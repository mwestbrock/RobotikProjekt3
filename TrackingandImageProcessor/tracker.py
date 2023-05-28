import math
import numpy as np
import cv2

class Tracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0


    def update(self, objects_rect):
        objects_bbs_ids = []

        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            same_object_detected = False
            for id, points in self.center_points.items():
                last_point = points[-1] if points else None
                if last_point is not None:
                    last_cx, last_cy = last_point
                    dist = math.hypot(cx - last_cx, cy - last_cy)

                    if dist < 25:
                        points.append((cx, cy))
                        objects_bbs_ids.append([x, y, w, h, id])
                        same_object_detected = True
                        break

            if not same_object_detected:
                self.center_points[self.id_count] = [(cx, cy)]
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1

        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        self.center_points = new_center_points.copy()
        return objects_bbs_ids



    def velocity(self, object_id, fps):
        if object_id in self.center_points:
            center_points = self.center_points[object_id]
            num_points = len(center_points)

            if num_points >= 2:
                # Convert center points to numpy array for Kalman filter input
                center_points = np.array(center_points, dtype=np.float32)

                # Create Kalman filter object
                kalman = cv2.KalmanFilter(4, 2)  # 4 states (x, y, dx, dy), 2 measurements (x, y)

                # Initialize Kalman filter parameters
                kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                    [0, 1, 0, 1],
                                                    [0, 0, 1, 0],
                                                    [0, 0, 0, 1]], dtype=np.float32)
                kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                     [0, 1, 0, 0]], dtype=np.float32)
                kalman.processNoiseCov = np.array([[1e-4, 0, 1e-2, 0],
                                                   [0, 1e-4, 0, 1e-2],
                                                   [1e-2, 0, 1, 0],
                                                   [0, 1e-2, 0, 1]], dtype=np.float32) * 0.01
                kalman.measurementNoiseCov = np.array([[1, 0],
                                                       [0, 1]], dtype=np.float32) * 1

                # Initialize Kalman filter with the first center point
                kalman.statePre = np.array([center_points[0][0], center_points[0][1], 0, 0], dtype=np.float32)
                kalman.statePost = np.array([center_points[0][0], center_points[0][1], 0, 0], dtype=np.float32)

                # Predict and update Kalman filter for each center point
                velocities = []
                for i in range(1, num_points):
                    kalman.correct(center_points[i])
                    predicted_state = kalman.predict()
                    velocities.append(predicted_state[2:4])

                # Calculate average velocity using Kalman filter predictions
                avg_velocity = (np.mean(velocities,axis=0) * 22) / fps  # Convert to cm/s

                return avg_velocity





