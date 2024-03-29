# ROS2 Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray

# Other Libraries
import cv2
from cv_bridge import CvBridge
import numpy as np

# Constants
NODE_NAME = "target_recognition"
IMG_CAPTURE = "image_capture_topic"
PROCESSED_IMG = "processed_image_topic"
COLOR = "color_topic"
DIRECTION = "direction_topic"
MIN_COLOR = 1
MAX_COLOR = 4
LOW_LIMIT = 0.01
HIGH_LIMIT = 0.5


class TargetHSV:
    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val


class TargetRecognition(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Subsricptions
        self.image_sub = self.create_subscription(
            Image, IMG_CAPTURE, self.image_callback, 1
        )
        self.color_sub = self.create_subscription(Int32, COLOR, self.color_callback, 10)

        # Publishers
        self.processed_image_pub = self.create_publisher(Image, PROCESSED_IMG, 1)
        self.direction_pub = self.create_publisher(Float64MultiArray, DIRECTION, 10)

        # Open CV Bridge
        self.cv_bridge = CvBridge()

        # Tracking Target HSV Value
        default = TargetHSV(min_val=(0, 0, 0), max_val=(255, 255, 255))
        green = TargetHSV(min_val=(59, 20, 225), max_val=(94, 159, 255))
        blue = TargetHSV(min_val=(92, 39, 202), max_val=(114, 255, 255))
        purple = TargetHSV(min_val=(138, 86, 139), max_val=(156, 255, 255))
        self.colors = [default, green, blue, purple]

        # Object State
        self.object_pixels = 0
        self.object_centroid = [0, 0]
        self.object_ratio = 0

        # Internal State
        self.cv_image = None
        self.target_color = 0
        self.start = 0.0

    # color_topic callback
    def color_callback(self, msg: Int32):
        color = msg.data
        if color >= MIN_COLOR and color <= MAX_COLOR:
            if color == 4:
                self.target_color = 0
                self.start = 0.0
                return
            self.target_color = color
            self.start = 1.0

    # image_capture_topic callback
    def image_callback(self, image):
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")

        if self.cv_image is not None:
            contour = self.extract_object_by_threshold(self.cv_image)

            if contour is not False:
                self.object_centroid = self.compute_centroid(contour)
                self.object_pixels = cv2.contourArea(contour)
                self.draw_contours(self.cv_image, contour)
                self.draw_target_bounding_box(self.cv_image, contour)
                self.draw_centroid(self.cv_image, self.object_centroid)
            else:
                self.object_pixels = 0

            self.processed_image_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
            )

            self.direction_control()

    # Draw Target Contour
    def draw_contours(self, cv_image, contours):
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

    # Draw Target Center Mass
    def draw_centroid(self, cv_image, centroid):
        cv2.circle(self.cv_image, self.object_centroid, 2, (0, 0, 255), 2)

    # Bounding box
    def draw_target_bounding_box(self, cv_image, contour):
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 2)

    #
    # 物体を検出する関数
    #
    def extract_object_by_threshold(self, cv_image):
        if self.target_color == 0:
            return False

        # 色空間の変換
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2値化
        color = self.colors[self.target_color]
        mask = cv2.inRange(hsv, color.min_val, color.max_val)

        # 穴埋め
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 最大面積を持つ領域を取得
        contour = self.select_biggest_object(mask)

        return contour

    #
    # 面積最大の領域を取得する関数
    #
    def select_biggest_object(self, mask):
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_contour_index = False
        biggest_contour_area = 0
        for n, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > biggest_contour_area:
                biggest_contour_area = area
                biggest_contour_index = n

        if biggest_contour_index is False:
            return False
        else:
            return contours[biggest_contour_index]

    #
    # 物体の重心を計算する関数
    #
    def compute_centroid(self, contour):
        moment = cv2.moments(contour)
        x = int(moment["m10"] / moment["m00"])
        y = int(moment["m01"] / moment["m00"])
        return [x, y]

    #
    # movement_controller への制御命令を送信する関数
    #
    def direction_control(self):
        msg = Float64MultiArray()
        forward_direction = 0.0
        pos_x_rate = 0.0
        if self.object_is_detected():
            pos_x_rate = self.compute_pos_x_rate()
            forward_direction = 1.0

        msg.data = [self.start, forward_direction, pos_x_rate]
        self.direction_pub.publish(msg)

    # 物体を検出したかどうか
    def object_is_detected(self):
        self.object_ratio = self.object_pixels / (
            self.cv_image.shape[0] * self.cv_image.shape[1]
        )

        return self.object_ratio > LOW_LIMIT and self.object_ratio < HIGH_LIMIT

    # X軸相対位置を計算する関数
    def compute_pos_x_rate(self):
        if self.cv_image is None or self.target_color == 0:
            return 0.0

        half_width = self.cv_image.shape[1] / 2.0
        pos_x_rate = (half_width - self.object_centroid[0]) / half_width

        return pos_x_rate


def main():
    rclpy.init()

    node = TargetRecognition()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
