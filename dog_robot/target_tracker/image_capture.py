# ROS2 Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Open CV Libraries
import cv2
from cv_bridge import CvBridge

NODE_NAME = "image_capture"
CAMERA_DEVICE = 0
CAP_PROP_FPS = 30
CAP_PROP_FRAME_WIDTH = 640
CAP_PROP_FRAME_HEIGHT = 480

IMAGE_CAP = "image_capture_topic"


class ImageCapture(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        self.image_publisher = self.create_publisher(Image, IMAGE_CAP, 1)

        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cv_bridge = CvBridge()

        self.capture = None

        self.init_camera()

    def init_camera(self):
        try:
            self.capture = cv2.VideoCapture(CAMERA_DEVICE)
            if not self.capture.isOpened():
                self.logger.warn("Unable to open the camera.")
                self.destroy_node()
            self.capture.set(cv2.CAP_PROP_FPS, CAP_PROP_FPS)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_WIDTH)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_PROP_FRAME_HEIGHT)
        except Exception as e:
            self.logger.error(e)

    def timer_callback(self):
        ret, image = self.capture.read()
        if ret:
            self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(image, "bgr8"))
        else:
            self.logger.error("Failed to capture image.")


def main():
    rclpy.init()

    node = ImageCapture()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
