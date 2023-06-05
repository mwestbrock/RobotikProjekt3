import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from imageTransformer import ImageTransformer


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            'detection_image_topic',
            10
        )

        self.image_processor = ImageTransformer()

    def image_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        transformed_frame, detection_area = self.image_processor.process_frame(frame)

        self.get_logger().info("image recieved")
        transformed_image_msg = self.cv_bridge.cv2_to_imgmsg(transformed_frame, encoding='bgr8')
        detection_area_msg = self.cv_bridge.cv2_to_imgmsg(detection_area, encoding='bgr8')

        #self.publisher.publish(transformed_image_msg)
        # Optional: Publish detection area as a separate topic
        self.publisher.publish(detection_area_msg)
        #Transformed Video anzeigen
        #cv2.imshow('Video',transformed_frame)
        #cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
