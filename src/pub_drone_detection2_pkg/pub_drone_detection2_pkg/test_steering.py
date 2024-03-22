import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

TOPIC_NAME = "topic_erreurFrame"
NODE_NAME = "TestNode"

class TestNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.error = -300
        self.publisher_ = self.create_publisher(Int32, TOPIC_NAME, 10)
        self.run()

    def run(self):
        increment = 1
        while rclpy.ok():
            msg = Int32()
            msg.data = self.error
            self.publisher_.publish(msg)
            self.error += increment
            if self.error == 300 or self.error == 0:
                increment *= -1  # Reverse the increment direction
            time.sleep(0.01)  # Delay of 0.5 seconds

def main(args=None):
    rclpy.init(args=args)
    test_steering = TestNode()
    try:
        rclpy.spin(test_steering)
        test_streering.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        test_streering.get_logger().info(f'Shutting down {NODE_NAME}...')
        test_streering.destroy_node()
        rclpy.shutdown()
        test_streering.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()
