import rclpy
from rclpy.node import Node 
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
import time
from tutorial_interfaces.msg import Gripper
""" Class SimpleController subscribes to the robot position and the desired robot position and publishes the robot command.
    Its purpose is to control the robot based on the current position and the desired position. """


class SimpleController(Node):
    def __init__(self):
        """ Constructor of SimpleController."""
    		
        super().__init__('controller_node')
        
        queue_size = 10

        self.subscription_pos = self.create_subscription(
            RobotPos,
            'robot_position',
            self.position_callback,
            queue_size
        )
        
        self.subscription_desired_pos = self.create_subscription(
            RobotPos,
            'robot_reference_position',
            self.desired_position_callback,
            queue_size
        )

        self.subscription_gripper = self.create_subscription(
            Gripper,
            'grip',
            self.gripper_callback,
            queue_size
        )
        
        self.publisher = self.create_publisher(
            RobotCmd, 
            'robot_command', 
            queue_size
        )
        
        publish_rate = 10  # 10Hz
        
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)
        self.grip = False
				
		# Positionen vom Typ RobotPos() mit default Konstruktor
        self.current_position = RobotPos()
        self.desired_position = RobotPos()
        #self.grip = False
    
    def gripper_callback(self, msg):
        """ Callback function for the gripper subscriber.
        This function is called whenever a new gripper position is received.
        """
        self.grip = msg.grip


    def position_callback(self, msg):
        """ Callback function for the robot position subscriber.
        This function is called whenever a new robot position is received.
        """
        self.current_position = msg

    def desired_position_callback(self, msg):
        """ Callback function for the desired robot position subscriber.
        This function is called whenever a new desired robot position is received.
        """
        self.desired_position = msg

    def publish_command(self):
        """ Publish the robot command."""
        cmd = self.calculate_control_signal()
        command = self.publisher.publish(cmd)

    def calculate_control_signal(self):
        """ Calculate the control signal.
        This function calculates the control signal based on the current position and the desired position.
        """
        kx = 3
        ky = 3
        kz = 3
        robot_command = RobotCmd()
        robot_command.vel_x = kx * (self.desired_position.pos_x - self.current_position.pos_x)
        robot_command.vel_y = ky * (self.desired_position.pos_y - self.current_position.pos_y)
        robot_command.vel_z = kz * (self.desired_position.pos_z - self.current_position.pos_z)
        robot_command.activate_gripper = self.grip
        return robot_command


def main(args=None):
    """ Main function of the simple controller."""
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

