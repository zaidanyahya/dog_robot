import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np


#
# HSVのしきい値を調べるクラス
#
class HSVControl(Node):
    #
    # コンストラクタ
    #
    def __init__(self):
        super().__init__("HSV_control")

        # 画像データのサブスクライバ
        self.image_subscription = self.create_subscription(
            Image, "cap_image", self.image_callback, 1
        )

        self.cv_bridge = CvBridge()

        self.min_val = np.zeros(3)
        self.max_val = np.zeros(3)

        cv2.namedWindow("Object Detection")
        cv2.createTrackbar("Hmin: ", "Object Detection", 0, 179, self.nothing)
        cv2.createTrackbar("Hmax: ", "Object Detection", 30, 179, self.nothing)
        cv2.createTrackbar("Smin: ", "Object Detection", 80, 255, self.nothing)
        cv2.createTrackbar("Smax: ", "Object Detection", 255, 255, self.nothing)
        cv2.createTrackbar("Vmin: ", "Object Detection", 0, 255, self.nothing)
        cv2.createTrackbar("Vmax: ", "Object Detection", 255, 255, self.nothing)

    def nothing(self, val):
        pass

    #
    # 画像データを受け取ったら呼び出される関数
    #
    def image_callback(self, image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        if cv_image is not None:
            mask = self.binarize_image(cv_image)
            pidx = np.where(mask != 255)
            for n in range(len(pidx[0])):
                cv_image[pidx[0][n], pidx[1][n]] = 0

            cv2.imshow("Object Detection", cv_image)
            cv2.waitKey(1)

    # しきい値による画像の2値化
    def binarize_image(self, image):
        # RGB色空間からHSV色空間への変換
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # しきい値を取得
        self.min_val[0] = cv2.getTrackbarPos("Hmin: ", "Object Detection")
        self.min_val[1] = cv2.getTrackbarPos("Smin: ", "Object Detection")
        self.min_val[2] = cv2.getTrackbarPos("Vmin: ", "Object Detection")
        self.max_val[0] = cv2.getTrackbarPos("Hmax: ", "Object Detection")
        self.max_val[1] = cv2.getTrackbarPos("Smax: ", "Object Detection")
        self.max_val[2] = cv2.getTrackbarPos("Vmax: ", "Object Detection")

        return cv2.inRange(hsv, self.min_val, self.max_val)


def main():
    rclpy.init()

    node = HSVControl()

    rclpy.spin(node)

    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
