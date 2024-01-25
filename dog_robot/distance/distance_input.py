# ROS2 Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# Constants
NODE_NAME = "distance_input"
DISTANCE_IN = "distanceIn_topic"
LIGHT_SENSOR = "/dev/rtlightsensor0"
DEFAULT_SENSOR_VALUE = [1000]
TIMER_PERIOD = 0.03  # seconds


# 光センサーの入力を読み取るノード
class DistanceInput(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # パブリシャーの設定
        self.publisher = self.create_publisher(Int32MultiArray, DISTANCE_IN, 10)

        # タイマーの設定
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    # タイマーコールバック関数
    def timer_callback(self):
        # Int32MultiArray 型のメッセージ作成
        msg = Int32MultiArray()
        # センサーの値が読み取れない場合のデフォルト値
        msg.data = DEFAULT_SENSOR_VALUE

        try:
            # 光センサーデータを読み取り、整数のリストに変換
            with open(LIGHT_SENSOR, "r") as f:
                msg.data = list(map(int, f.read().split()))
        except:
            self.logger.error(f"Light sensor: {LIGHT_SENSOR} is not found")

        # メッセージをパブリッシュ
        self.publisher.publish(msg)


def main():
    rclpy.init()

    node = DistanceInput()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if "__name__" == "__main__":
    main()
