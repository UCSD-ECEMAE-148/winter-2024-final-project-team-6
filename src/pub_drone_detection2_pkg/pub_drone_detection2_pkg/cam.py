import cv2
import depthai as dai   
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

NODE_NAME = "camera_node"

class CameraNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        # Create a pipeline
        pipeline = dai.Pipeline()

        # Create a color camera node
        cam = pipeline.createColorCamera()
        cam.setPreviewSize(1920, 1080)
        cam.setInterleaved(False)

        # Create an XLink output node for the color camera
        xout = pipeline.createXLinkOut()
        xout.setStreamName("preview")
        cam.preview.link(xout.input)

        self.pub = self.create_publisher(Image, "camera", 10)
        self.run()

    def run(self):
        # Connect to the device and start the pipeline
        with dai.Device(self.pipeline) as device:
            # Output queue for the color camera stream
            q = device.getOutputQueue(name="preview", maxSize=4, blocking=False)

            while rclpy.ok():
                frame = q.get()  # Get a frame from the queue
                img = frame.getCvFrame()  # Convert the frame to an OpenCV format

                cv2.imshow("OAK-D Camera", img)  # Display the frame
                self.pub.publish(img)  # Publish the frame to the topic

                if cv2.waitKey(1) == ord("q"):  # Exit on Q key
                    break


def main(args=None):
    rclpy.init(args=args)
    cam_node = CameraNode()
    try:
        rclpy.spin(cam_node)
        cam_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        cam_node.get_logger().info(f"Shutting down {NODE_NAME}...")

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        cam_node.destroy_node()
        rclpy.shutdown()
        cam_node.get_logger().info(f"{NODE_NAME} shut down successfully.")


if __name__ == "__main__":
    main()
