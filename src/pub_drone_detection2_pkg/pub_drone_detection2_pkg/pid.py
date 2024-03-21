import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
import time
import os
from std_msgs.msg import Int32

NODE_NAME = 'pid_node'
ERROR_TOPIC_NAME = '/topic_erreurFrame'
ACTUATOR_TOPIC_NAME = '/cmd_vel' # These are basically creating constants that once set are not able to be changed


class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME) # We are calling the constructor (Initilizes Object state, setting initial values for attributes) of a super class (Node) which passes the "NODE_NAME" as the arguement 
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.error_subscriber = self.create_subscription(Int32, ERROR_TOPIC_NAME, self.controller, 10)

        # # Default actuator values
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('Kp_steering', 1),
        #         ('Ki_steering', 0),
        #         ('Kd_steering', 0),
        #         ('error_threshold', 0.15),
        #         ('zero_throttle',0.0),
        #         ('max_throttle', 0.2),
        #         ('min_throttle', 0.1),
        #         ('max_right_steering', 1.0),
        #         ('max_left_steering', -1.0)
        #     ])

        self.Kp = 0.1 # between [0,1]
        self.Ki = 0 # between [0,1]
        self.Kd = 0 # between [0,1]
        
        self.zero_throttle = 0 # between [-1,1] but should be around 0
        self.max_throttle = 0.2 # between [-1,1]
        self.min_throttle = 0 # between [-1,1]
        self.max_right_steering = 0.5 # between [-1,1]
        self.max_left_steering = -0.5 # between [-1,1]
        self.error_threshold = 0.15

        # initializing PID control
        self.Ts = float(1/20)
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8
        
        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )

    def controller(self, data):
        self.get_logger().info(f'we got into controller function')
        # setting up PID control
        self.ek = data.data

        # Throttle gain scheduling (function of error)
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Steering PID terms
        self.proportional_error = self.Kp * self.ek
        self.get_logger().info(f'proportional_error: {self.proportional_error}')

        self.derivative_error = self.Kd * (self.ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * self.ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

        # Publish values
        try:
            # publish control signals
            self.twist_cmd.angular.z = float(steering_float)
            self.twist_cmd.linear.x = float(throttle_float)
            self.twist_publisher.publish(self.twist_cmd)

            # shift current time and error values to previous values
            self.ek_1 = self.ek

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 


def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
