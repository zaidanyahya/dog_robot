# ROS2 Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Open CV Libraries
import cv2
from cv_bridge import CvBridge

# Constants
NODE_NAME = "target_monitor"
PROCESSED_IMG = "processed_image_topic"


class TargetMonitor(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.image_subscription = self.create_subscription(
            Image, PROCESSED_IMG, self.image_callback, 1
        )

        self.cv_bridge = CvBridge()

    def image_callback(self, image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        if cv_image is not None:
            cv2.imshow("Dog Robot POV Monitoring", cv_image)
            cv2.waitKey(1)


def main():
    rclpy.init()

    node = TargetMonitor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
