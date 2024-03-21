import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import time
import numpy as np
#from pub_drone_detection2_pkg.msg import CameraError
from std_msgs.msg import Int32
import requests
import base64

NODE_NAME = "topic_erreurFrame"
ROBOFLOW_MODEL = "drone-tracking-fztzm"
ROBOFLOW_API_KEY = "0RmqstHKjwDcunOH9wus"
ROBOFLOW_SIZE = 416

upload_url = "".join([
    "http://0.0.0.0:9001/" + ROBOFLOW_MODEL,
    "?access_token=" + ROBOFLOW_API_KEY,
    "&name=YOUR_IMAGE.jpg"
])

video = cv2.VideoCapture(2)

class JetsonInferenceNode(Node):

    error = 0
    prev_error = 0
    _x = 0
    _y = 0
    confidence = 0
    repeat_counter = 0

    def __init__(self):
        super().__init__(NODE_NAME)
        
        self.publisher_ = self.create_publisher(Int32, NODE_NAME, 10)

        self.run()

    def run(self):
        # Running the model and displaying the video output with detections
        while rclpy.ok():

            ret, img = video.read()

            # Resize (while maintaining the aspect ratio) to improve speed and save bandwidth
            height, width, channels = img.shape
            scale = ROBOFLOW_SIZE / max(height, width)
            img = cv2.resize(img, (round(scale * width), round(scale * height)))

            # # Encode image to base64 string
            # retval, buffer = cv2.imencode('.jpg', img)
            # img_str = base64.b64encode(buffer)

            # # Get predictions from Roboflow Infer API
            # resp = requests.post(upload_url, data=img_str, headers={
            #     "Content-Type": "application/x-www-form-urlencoded"
            # }, stream=True).json()['predictions']

            # # Draw all predictions
            # for prediction in resp:
            #     self.writeOnStream(prediction['x'], prediction['y'], prediction['width'], prediction['height'],
            #                 prediction['class'],
            #                 img)

            cv2.imshow("frame", img)
            # # log the x y coords of the drone
            # self.prev_highest_confidence = 0
            # for p in predictions:
                
            #     self.confidence = float(p.confidence)
            #     if self.confidence > self.prev_highest_confidence:
            #         self.prev_highest_confidence = self.confidence
            #         self._x = int(p.x)
            #         self._y = int(p.y)

            #     self.get_logger().info(f'x: {self._x}, y: {self._y}, Confidence: {self.confidence}, Highest confidence: {self.prev_highest_confidence}')

            # # Get the center coordinates of the frame
            # #if 
            # frame_height, frame_width, num_channels = frame.shape
            # center_x = frame_width // 2
            # center_y = frame_height // 2


            # ### CALCULATE THE ERROR FOR STEERING ###
            # self.error = self._x - center_x
            # self.get_logger().info(f'Error: {self.error}')
            
            # # stop steering (persisting error) if we lose the drone
            # if (self.error == self.prev_error):
            #     self.repeat_counter+=1
            # else:
            #     self.repeat_counter = 0
            #     self.prev_error = self.error

            # # 10 repeats in a row
            # if (self.repeat_counter > 9):
            #     self.repeat_counter = 0
            #     self.error = 0

            # msg = Int32()
            # msg.data = self.error
            # self.publisher_.publish(msg)

            # # Draw a circle at the center coordinates
            # cv2.circle(frame, (center_x, center_y), radius=10, color=(0, 255, 0), thickness=2)

            # # Draw a red vertical line that splits the frame in half
            # cv2.line(frame, (center_x, 0), (center_x, frame_height), color=(0, 0, 255), thickness=2)

            # # Draw a line between the center coordinates and the drone coordinates
            # cv2.line(frame, (center_x, center_y), (self._x, self._y), color=(255, 0, 0), thickness=2)

            # # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # # Display the grayscale frame
            # # cv2.imshow("frame", gray_frame)
            # cv2.imshow("frame", frame)
            # t = time.time() - t0
            # self.get_logger().info(f'FPS: {1/t}')
            

            # # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
            if cv2.waitKey(1) == ord('q'):
                 break
            

    def writeOnStream(x, y, width, height, className, frame):
        # Draw a Rectangle around detected image
        cv2.rectangle(frame, (int(x - width / 2), int(y + height / 2)), (int(x + width / 2), int(y - height / 2)),
                    (255, 0, 0), 2)

        # Draw filled box for class name
        cv2.rectangle(frame, (int(x - width / 2), int(y + height / 2)), (int(x + width / 2), int(y + height / 2) + 35),
                    (255, 0, 0), cv2.FILLED)

        # Set label font + draw Text
        font = cv2.FONT_HERSHEY_DUPLEX

        cv2.putText(frame, className, (int(x - width / 2 + 6), int(y + height / 2 + 26)), font, 0.5, (255, 255, 255), 1)


def main(args=None):
    rclpy.init(args=args)
    jetson_inference = JetsonInferenceNode()
    try:
        rclpy.spin(jetson_inference)
        jetson_inference.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        jetson_inference.get_logger().info(f'Shutting down {NODE_NAME}...')
        

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        jetson_inference.destroy_node()
        rclpy.shutdown()
        jetson_inference.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()
