
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from tutorial_interfaces.msg import Detection, Gripper
import time

""" Class GripperNode subscribes to the robot position and the detected object and publishes the robot command."""

class GripperNode(Node):
    """Gripper Node class.
    Attributes:
        publisher_reference_pos (Publisher): Publisher for the robot reference position
        publisher_gripper (Publisher): Publisher for the gripper command
        subscription_robot_pos (Subscription): Subscriber for the robot position
        subscription_object (Subscription): Subscriber for the detected object
        init (bool): True if the robot is in the initial position, False otherwise
        current_position (RobotPos): Current position of the robot
        lock_gripper (bool): True if the gripper is locked, False otherwise
        x_start (float): Initial position in x direction
        y_start (float): Initial position in y direction
        z_start (float): Initial position in z direction
        x_einhorn (float): Unicorn position in x direction
        x_katze (float): Cat position in x direction
        y_box (float): Box position in y direction
        z_box (float): Box position in z direction
        z_grip (float): Grip position in z direction
        distanceUnicorn (float): Distance to unicorn pick up position
        catDistance (float): Distance to cat pick up position

        
    """
    def __init__(self):
        """ Constructor of GripperNode."""
        super().__init__('gripper_node')
        
        self.publisher_reference_pos = self.create_publisher(
            RobotPos,
            'robot_reference_position',
            10
        )

        self.publisher_gripper = self.create_publisher(
            Gripper,
            'grip',
            10
        )

        self.subscription_robot_pos = self.create_subscription(
            RobotPos,
            'robot_position',
            self.position_callback,
            10
        )

        self.subscription_object = self.create_subscription(
            Detection,
            'detection',
            self.object_callback,
            10
        )
        #Robot Constants
        self.init = False

        self.current_position = None
        self.lock_gripper = False

        self.x_start = 0.04
        self.y_start = 0.02
        self.z_start = 0.068

        self.x_einhorn = 0.01
        self.x_katze = 0.07

        self.y_box = 1.0
        self.z_box = 0.03

        self.z_grip = 0.072

        self.distanceUnicorn = 0.20
        self.catDistance = 0.15
        
        self.move_init_position()
        self.get_logger().info('Gripper node started')

    def position_callback(self, msg):
        """ Callback function for the robot position subscriber.
        This function is called whenever a new robot position is received.
            Args:
                msg (RobotPos): Current position of the robot
        """
        self.current_position = msg

    def publish_reference_pos(self, x, y, z):
        """ Publishes the reference position of the robot.
            Args:
                x (float): Position in x direction
                y (float): Position in y direction  
                z (float): Position in z direction
        """
        msg = RobotPos()
        msg.pos_x = x
        msg.pos_y = y       
        msg.pos_z = z
        self.publisher_reference_pos.publish(msg)

    def publish_gripper(self, grip):
        """ Publishes the gripper command.
            Args:
                grip (bool): True to grip, False to release
        """
        msg = Gripper()
        self.get_logger().info('Gripper: {}'.format(msg))
        msg.grip = grip
        self.publisher_gripper.publish(msg)

    def pixel_to_meter(self, pixel):
        """ Converts pixel to meter. 0.01m = 40px
            Args:
                pixel (float): Pixel to convert
                    
            Returns:
                float: Converted pixel
        """
        
        return pixel * 0.00025
        
    
    def calculate_object_coords(self, object_y):
        """ Calculates Camera coordinates to robot coordinates in y direction
            Args:
                object_y (float): Object position (grippoint) in y direction
                    
            Returns:
                float: Calculated object position (grippoint)
        """
        if object_y > 120 :
            delta_y = object_y - 120
            delta_y = self.pixel_to_meter(delta_y)
            return -delta_y
        elif object_y < 120 :
            delta_y = 120 - object_y
            delta_y = self.pixel_to_meter(delta_y)
            return delta_y
        else:
            return 0

    def move_init_position(self):
        """ Moves the robot to the initial position."""
        if self.init == False:
            self.init = True
            self.publish_reference_pos(self.x_start , self.y_start, self.z_start)

    def move_idle_position(self):
        """ Moves the robot to the idle position."""
        self.publish_reference_pos(self.x_start, self.y_start, self.z_start)

    #CAT

    def move_cat_position(self, y_object):
        """ Moves the robot to the grip position of the cat in x and y direction.
            Args:
                y_object (float): Object position in y direction
        """
        delta_y = self.calculate_object_coords(y_object)
        self.publish_reference_pos(self.x_katze, self.y_start + delta_y, self.z_start)
    
    def move_cat_up(self, y_object):
        """ Moves the robot upwards in z direction.
            Args:
                y_object (float): Object position in y direction
        """
        delta_y = self.calculate_object_coords(y_object)
        self.publish_reference_pos(self.x_katze, self.y_start + delta_y, self.z_box)
    
    def move_cat_grab(self, y_object):
        """ Moves the robot downwards in z direction to grip the cat.
            Args:
                y_object (float): Object position in y direction
        """
        delta_y = self.calculate_object_coords(y_object)
        self.publish_reference_pos(self.x_katze, self.y_start + delta_y, self.z_grip)
    
    def move_cat_to_box(self):
        """ Moves the robot to the Cat box position."""
        self.publish_reference_pos(self.x_katze, self.y_box, self.z_box)

    #UNICORN
    
    def move_unicorn_position(self, y_object):
        """ Moves the robot to the unicorn grip position in x and y direction.
            Args:
                y_object (float): Object position in y direction
        """
        delta_y = self.calculate_object_coords(y_object)
        self.publish_reference_pos(self.x_einhorn, self.y_start + delta_y, self.z_start)

    def move_unicorn_up(self, y_object):
        """ Moves the unicorn upwards in z direction.
            Args:
                y_object (float): Object position in y direction
        """
        delta_y = self.calculate_object_coords(y_object)
        self.publish_reference_pos(self.x_einhorn, self.y_start + delta_y, self.z_box)
    
    def move_unicorn_grab(self, y_object):
        """ Moves the robot downwards in z direction to grip the unicorn.
            Args:
                y_object (float): Object position in y direction
        """
        delta_y = self.calculate_object_coords(y_object)
        self.publish_reference_pos(self.x_einhorn, self.y_start + delta_y, self.z_grip)
    
    def move_unicorn_to_box(self):
        """ Moves the robot to the unicorn box position."""
        self.publish_reference_pos(self.x_einhorn, self.y_box, self.z_box)
    
    def gripper_off(self):
        """ Turns the gripper off."""
        self.publish_gripper(False)
    
    def gripper_on(self):
        """ Turns the gripper on."""
        self.publish_gripper(True)

    def object_callback(self, msg):
        """ Callback function for the detected object subscriber.
        This function is called whenever a new detected object is received. 
        The function checks if the detected object is an unicorn (1) or a cat (0) and moves the robot to the object.
            Args:
                msg (Detection): Detected object

        """
        self.get_logger().info('Object detected')
        object_id = msg.id
        object_x = msg.gpx
        object_y = msg.gpy
        object_classification = msg.classification
        object_velocity = msg.velocity
        
        #Object information for debugging purposes
        """
        self.get_logger().info('Object ID: {}'.format(object_id))
        self.get_logger().info('Object X: {}'.format(object_x))
        self.get_logger().info('Object Y: {}'.format(object_y))
        self.get_logger().info('Classification: {}'.format(object_classification))
        self.get_logger().info('Object Velocity: {}'.format(object_velocity))"""
        
        #Cat: 0
        if object_classification == 0 and not self.lock_gripper:  
            self.lock_gripper=True
            self.get_logger().info('Cat detected')
            time1 = time.time()
            self.move_cat_position(object_y)
            time2 = time.time()
            deltaTime = time2 - time1
            waitTime = (self.catDistance * 100)/ object_velocity 
            waitTime = waitTime - deltaTime
            self.get_logger().info('Wait time: {}'.format(waitTime))
            time.sleep(waitTime)
            self.get_logger().info('Moving to cat')
            self.gripper_on()
            self.move_cat_grab(object_y)
            time.sleep(2)
            self.move_cat_up(object_y)
            time.sleep(2)
            self.move_cat_to_box()
            time.sleep(4)
            self.gripper_off()
            time.sleep(1)
            self.move_idle_position()
            self.lock_gripper=False
            self.get_logger().info('Cat sorted')
            
        #Unicorn: 1
        elif object_classification == 1 and not self.lock_gripper:  # Einhorn
            self.lock_gripper=True
            self.get_logger().info('Unicorn detected')
            time1 = time.time()
            self.move_unicorn_position(object_y)
            time2 = time.time()
            deltaTime = time2 - time1
            waitTime = (self.distanceUnicorn*100)/ object_velocity
            waitTime = waitTime - deltaTime
            self.get_logger().info('Wait time: {}'.format(waitTime))
            time.sleep(waitTime)
            self.get_logger().info('Moving to unicorn')
            self.gripper_on()
            self.move_unicorn_grab(object_y)
            time.sleep(2)
            self.move_unicorn_up(object_y)
            time.sleep(2)
            self.move_unicorn_to_box()
            time.sleep(4)
            self.gripper_off()
            time.sleep(1)
            self.move_idle_position()
            self.lock_gripper=False
            self.get_logger().info('Unicorn sorted')


def main(args=None):
    """ Main function of the gripper node. """""
    rclpy.init(args=args)
    gripper_node = GripperNode()
    rclpy.spin(gripper_node)
    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
