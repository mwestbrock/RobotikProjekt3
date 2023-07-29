import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from . import imageTransformer

class ImageProcessorNode(Node):
    """
    A ROS2 node that processes image frames and publishes cropped images.

    Subscribes to the "video_frames" topic for incoming image messages, processes them using the
    ImageProcessor class, and publishes the cropped images to the "cropped_image" topic.

    Attributes:
        subscription: A subscription object for subscribing to the "video_frames" topic.
        publisher: A publisher object for publishing cropped images to the "cropped_image" topic.
        image_processor: An instance of the ImageProcessor class for image processing operations.
        cv_bridge: A CvBridge object for converting between ROS Image messages and OpenCV images.
    """

    def __init__(self):
        """
        Initialize the ImageProcessorNode.
        """
        super().__init__("image_processor_node")
        self.subscription = self.create_subscription(
            Image,
            "video_frames",
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image, 
            "cropped_image", 
            10)
        self.image_processor = imageTransformer.ImageProcessor()
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        """
        Callback function for processing incoming image messages.

        Args:
            msg: The incoming Image message.

        Performs image processing operations on the received image, including resizing the frame,
        detecting Aruco markers, transforming to bird's eye view, and cropping the warped image.
        Publishes the cropped image to the "cropped_image" topic.

        Note:
            This function assumes that the necessary methods for image processing are implemented
            in the ImageProcessor class.
        """
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = self.image_processor.resize_frame(cv_image, 100)

        arucoMarker = self.image_processor.detect_markers(cv_image)
        if len(arucoMarker) == 4:
            birdEyeImage = self.image_processor.transform_to_birds_eye_view(cv_image, arucoMarker)
            cropped_image = self.image_processor.crop_warped_image(birdEyeImage)

            if cropped_image is not None:
                cropped_image_msg = self.cv_bridge.cv2_to_imgmsg(cropped_image, encoding="bgr8")
                self.publisher.publish(cropped_image_msg)
                #self.get_logger().info("Published cropped image.")
                #cv2.imshow("Cropped Image", cropped_image)
                #cv2.waitKey(1)

def main(args=None):
    """
    Main entry point of the program.

    Args:
        args: Command-line arguments.

    Initializes the ROS2 node, creates an instance of the ImageProcessorNode,
    and spins the node until shutdown.
    """
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
