import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(
            Image, 
            'video_frames', 
            10)
        self.timer_ = self.create_timer(
            1.0/30, 
            self.publish_frame)  # 30 FPS
        self.cv_bridge_ = CvBridge()
        
        test_video = True   #True wenn mit Video gearbeitet wird
        if test_video:
            self.video_capture_=cv2.VideoCapture('/home/mbird/Videos/CatsUnicorns.mp4')
        else:
            self.video_capture_ = cv2.VideoCapture(0)  # Öffne Kamera 0

    def publish_frame(self):


        ret, frame = self.video_capture_.read()
        if ret:
            msg = self.cv_bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
