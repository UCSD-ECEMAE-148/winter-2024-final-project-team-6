from launch import LaunchDescription
from launch_ros.actions import Node
#from adafruit_servo_node_custom import adafruit_servo_node_custom



def generate_launch_description():
    ld = LaunchDescription()

    inference_node = Node(
        package="pub_drone_detection2_pkg",
        executable="jetson_inference.py",
        output="screen",
    )
    ld.add_action(inference_node)


    return ld
