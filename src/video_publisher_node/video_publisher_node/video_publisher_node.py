import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    """
    A ROS2 node that publishes video frames as Image messages.

    Subscribes to a video source (either a video file or a camera) and publishes the frames
    as Image messages on the "video_frames" topic.

    Attributes:
        publisher_: A publisher object for publishing Image messages.
        timer_: A timer object for periodically calling the publish_frame method.
        cv_bridge_: A CvBridge object for converting between OpenCV images and ROS Image messages.
        video_capture_: A VideoCapture object for capturing video frames from a source (video file or camera).
    """

    def __init__(self):
        """
        Initialize the VideoPublisher node.
        """
        super().__init__('video_publisher_node')
        self.publisher_ = self.create_publisher(
            Image, 
            'video_frames', 
            10)
        self.timer_ = self.create_timer(
            1.0/30, 
            self.publish_frame)  # 30 FPS
        self.cv_bridge_ = CvBridge()
        
        test_video = False   #True if video file, False if camera

        if test_video == True:
            self.video_capture_= cv2.VideoCapture('/home/mbird/Downloads/AruCoGemischt.mp4')
        else:
            self.video_capture_ = cv2.VideoCapture(0)  # Öffne Kamera 0

    def publish_frame(self):
        """
        Callback function for capturing and publishing video frames.

        Captures a frame from the video source, converts it to an Image message,
        and publishes it on the "video_frames" topic.
        """
        ret, frame = self.video_capture_.read()
        if ret:
            msg = self.cv_bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            #self.get_logger().info('Publishing video frame')
            #cv2.imshow("Frame", frame)
            #cv2.waitKey(1)  

def main(args=None):
    """
    Main entry point of the program.

    Args:
        args: Command-line arguments.

    Initializes the ROS2 node, creates an instance of the VideoPublisher node,
    and spins the node until shutdown.
    """
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
