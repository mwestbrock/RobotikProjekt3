
from launch import LaunchDescription
from launch_ros.actions import Node
import os

"""" Launch file for all Ros2 Nodes """
def generate_launch_description():
    ld = LaunchDescription()
    video_publisher_node = Node(
        package='video_publisher_node',
        executable='video_publisher_node',
        )

    image_processor_node = Node(
        package='image_processor_node',
        executable='image_processor_node',  
        )

    detection_node = Node(
        package='detection_node',
        executable='detector',
        )
    
    gripper_node = Node(
        package='gripper_node',
        executable='gripper_node',
        )

    simple_controller = Node(
        package='simple_controller',
        executable='controller',
        )
    
    ld.add_action(video_publisher_node)
    ld.add_action(image_processor_node)
    ld.add_action(detection_node)
    ld.add_action(gripper_node)
    ld.add_action(simple_controller)

    return ld        
    

