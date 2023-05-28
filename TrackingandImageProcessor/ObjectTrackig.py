#Class which gets a Bonding Box and Tracks it through the Video, also calculates the Velocity and set a ID to the Object
import cv2
import numpy as np

# """
class ObjectTracking:

     def __init__(self, boundingBox, frame, ID):
         self.boundingBox = boundingBox
         self.frame = frame
         self.ID = ID
         self.tracker = cv2.TrackerMIL_create()
         self.tracker.init(self.frame, self.boundingBox)
         self.previousPosition = None
         self.velocity = None

     def update(self, frame):
        success, box = self.tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            currentPosition = np.array([x, y])
        if self.previousPosition is not None:
            self.velocity = np.linalg.norm(currentPosition - self.previousPosition)
            print("Geschwindigkeit: ", self.velocity)
            self.previousPosition = currentPosition
        return frame

     def getID(self):
         return self.ID

     def getVelocity(self):
         return self.velocity

     def getBoundingBox(self):
         return self.boundingBox

     def getTracker(self):
            return self.tracker

     def getPreviousPosition(self):
         return self.previousPosition

     def getFrame(self):
         return self.frame

     def setBoundingBox(self, boundingBox):
         self.boundingBox = boundingBox

     def setID(self, ID):
         self.ID = ID

     def setVelocity(self, velocity):
         self.velocity = velocity

     def setTracker(self, tracker):
         self.tracker = tracker

     def setPreviousPosition(self, previousPosition):
         self.previousPosition = previousPosition

     def setFrame(self, frame):
         self.frame = frame

     def __str__(self):
         return "ID: " + str(self.ID) + " Velocity: " + str(self.velocity) + " BoundingBox: " + str(self.boundingBox)

     def __repr__(self):
         return "ID: " + str(self.ID) + " Velocity: " + str(self.velocity) + " BoundingBox: " + str(self.boundingBox)

     def __eq__(self, other):
         if isinstance(other, ObjectTracking):
             return self.ID == other.ID
         return False

     def __hash__(self):
         return hash(self.ID)

     def __lt__(self, other):
         if isinstance(other, ObjectTracking):
             return self.ID < other.ID
         return False

     def __le__(self, other):
         if isinstance(other, ObjectTracking):
             return self.ID <= other.ID
         return False

     def __gt__(self, other):
         if isinstance(other, ObjectTracking):
             return self.ID > other.ID
         return False

     def __ge__(self, other):
         if isinstance(other, ObjectTracking):
             return self.ID >= other.ID
         return False

     def __ne__(self, other):
         if isinstance(other, ObjectTracking):
             return self.ID != other.ID
         return False

     def __cmp__(self, other):
         if isinstance(other, ObjectTracking):
             return self.ID.__cmp__(other.ID)
         return False

     def __hash__(self):
         return hash(self.ID)


"""
#
import cv2
import numpy as np


# Read a Video
cap = cv2.VideoCapture('Video.mp4')
#
# # Read the first frame
ret, frame = cap.read()
#
# # Select ROI
r = cv2.selectROI(frame)
#
# # Create a ObjectTracking Object
objectTracking = ObjectTracking(r, frame, 1)
#
# # Create a list of ObjectTracking Objects
objectTrackingList = []
objectTrackingList.append(objectTracking)
#
# # Create a list of IDs
IDList = []
IDList.append(1)
#
# # Create a list of Velocities
velocityList = []
velocityList.append(0)
#
# # Create a list of Trackers
trackerList = []
trackerList.append(objectTracking.getTracker())
#
# # Create a list of Previous Positions
previousPositionList = []
previousPositionList.append(objectTracking.getPreviousPosition())
#
# # Create a list of Frames
frameList = []
frameList.append(objectTracking.getFrame())
#
# # Initialize the MultiTracker
multiTracker = cv2.MultiTracker_create()
#
# # Add the ObjectTracking Object to the MultiTracker
multiTracker.add(objectTracking.getTracker(), objectTracking.getFrame(), objectTracking.getID())
#
# # Initialize the Counter
counter = 1
#
# # Initialize the ID
ID = 1
#
# # Initialize the Velocity
velocity = 0
#
# # Initialize the Previous Position
previousPosition = None
#
# # Initialize the Frame
frame = None
#
# # Initialize the BoundingBox
boundingBox = None
#
# # Initialize the Tracker
tracker = None
#
# # Initialize the ObjectTracking Object
objectTracking = None"""