from launch import LaunchDescription
from launch_ros.actions import Node
#from adafruit_servo_node_custom import adafruit_servo_node_custom



def generate_launch_description():
    ld = LaunchDescription()
#    drone_detection_node = Node(
#        package="pub_drone_detection2_pkg",
#        executable="drone_detection2.py",
#        output="screen",
#    )
#    ld.add_action(drone_detection_node)

    pid_node = Node(
        package="pub_drone_detection2_pkg",
        executable="pid.py",
        output="screen",
    )
    ld.add_action(pid_node)

    vesc_twist_node = Node(
        package="pub_drone_detection2_pkg",
        executable="vesc_twist_node_custom.py",
        output="screen",
    )
    ld.add_action(vesc_twist_node)

#    adafruit_servo_node_custom = Node(
#        package="pub_drone_detection2_pkg",
#        executable="adafruit_servo_node_custom.py",
#        output="screen",
#    )
#    ld.add_action(adafruit_servo_node_custom)

    test_steering = Node(
        package="pub_drone_detection2_pkg",
        executable="test_steering.py",
        output="screen",
    )
    ld.add_action(test_steering)

    inference_node = Node(
        package="pub_drone_detection2_pkg",
        executable="jetson_inference.py",
        output="screen",
    )
    ld.add_action(inference_node)


    return ld
