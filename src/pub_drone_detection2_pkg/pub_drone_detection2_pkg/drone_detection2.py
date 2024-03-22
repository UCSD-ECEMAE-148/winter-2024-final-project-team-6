import rclpy
from rclpy.node import Node
from roboflowoak import RoboflowOak
import cv2
from cv_bridge import CvBridge
import time
import numpy as np
#from pub_drone_detection2_pkg.msg import CameraError
from std_msgs.msg import Int32

NODE_NAME = "topic_erreurFrame"

class RoboflowOakNode(Node):

    error = 0
    prev_error = 0
    _x = 0
    _y = 0
    confidence = 0
    repeat_counter = 0
    times = []

    def __init__(self):
        super().__init__(NODE_NAME)

        # Instantiate an object (rf) with the RoboflowOak module
        self.rf = RoboflowOak(model="drone-tracking-fztzm", confidence=0.25, overlap=0.5,
                               version="1", api_key="0RmqstHKjwDcunOH9wus", rgb=True,
                               depth=False, device=None, blocking=True)
        
        self.publisher_ = self.create_publisher(Int32, NODE_NAME, 10)

        self.run()

    def run(self):
        # Running the model and displaying the video output with detections
        while rclpy.ok():

            t0 = time.time()            # The rf.detect() function runs the model inference
            result, frame, raw_frame, depth = self.rf.detect()
            predictions = result["predictions"]

            # Reduce the resolution of the frame while preserving aspect ratio
            #### FRAME RESIZING ----------------------------------------------
            max_width = 256
            max_height = 128
            frame_width, frame_height, _ = frame.shape
            aspect_ratio = frame_width / frame_height

            if aspect_ratio > max_width / max_height:
                new_width = max_width
                new_height = int(new_width / aspect_ratio)
            else:
                new_height = max_height
                new_width = int(new_height * aspect_ratio)

            frame = cv2.resize(frame, (new_width, new_height))

            #### --------------------------------------------------------------

            # log the x y coords of the drone
            self.prev_highest_confidence = 0
            for p in predictions:
                
                self.confidence = float(p.confidence)
                if self.confidence > self.prev_highest_confidence:
                    self.prev_highest_confidence = self.confidence
                    self._x = int(p.x)
                    self._y = int(p.y)

                # self.get_logger().info(f'x: {self._x}, y: {self._y}, Confidence: {self.confidence}, Highest confidence: {self.prev_highest_confidence}')

            # Get the center coordinates of the frame
            #if 
            frame_height, frame_width, num_channels = frame.shape
            center_x = frame_width // 2
            center_y = frame_height // 2

            ### CALCULATE THE ERROR FOR STEERING ###
            self.error = self._x - center_x
            # self.get_logger().info(f'Error: {self.error}')
            
            # stop steering (persisting error) if we lose the drone
            if (self.error == self.prev_error):
                self.repeat_counter+=1
            else:
                self.repeat_counter = 0
                self.prev_error = self.error

            # 10 repeats in a row
            if (self.repeat_counter > 9):
                self.repeat_counter = 0
                self.error = 0

            msg = Int32()
            msg.data = self.error
            self.publisher_.publish(msg)

            # Draw a circle at the center coordinates
            cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 255, 0), thickness=2)

            # Draw a red vertical line that splits the frame in half
            cv2.line(frame, (center_x, 0), (center_x, frame_height), color=(0, 0, 255), thickness=2)

            # Draw a line between the center coordinates and the drone coordinates
            cv2.line(frame, (center_x, center_y), (self._x, self._y), color=(255, 0, 0), thickness=2)

            # Convert the frame to grayscale
            # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Display the grayscale frame
            # cv2.imshow("frame", gray_frame)
            cv2.imshow("frame", frame)

            t = time.time() - t0

            # add dt to the list
            self.times.append(t)

            # find avg dt from times
            avg_dt = sum(self.times) / len(self.times)
            self.get_logger().info(f'Average dt: {avg_dt}')

            self.get_logger().info(f'dt: {t}')
            self.get_logger().info(f'FPS: {1/t}')
            

            # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
            if cv2.waitKey(1) == ord('q'):
                 break

def main(args=None):
    rclpy.init(args=args)
    drone_detection2 = RoboflowOakNode()
    try:
        rclpy.spin(drone_detection2)
        drone_detection2.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:

        drone_detection2.get_logger().info(f'Shutting down {NODE_NAME}...')

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        drone_detection2.destroy_node()
        rclpy.shutdown()
        drone_detection2.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()
