# ROS2 Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray

# Constants
NODE_NAME = "distance_assessment"
DISTANCE_IN = "distanceIn_topic"
DISTANCE_AS = "distanceAs_topic"
LIGHT_SENSOR = "/dev/rtlightsensor0"
LINE = 100


# センサー入力に基づいて距離を評価し、ブレーキ状態を公表するノード
class DistanceAssessment(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # サブスクリプションの設定
        self.subscription = self.create_subscription(
            Int32MultiArray, DISTANCE_IN, self.light_callback, 10
        )

        # パブリシャーの設定
        self.publisher = self.create_publisher(Bool, DISTANCE_AS, 10)

    # "distanceIn_topic" からのメッセージを処理するコールバック関数
    def light_callback(self, msg: Int32MultiArray):
        brake = Bool()

        # 光センサーデータの最大値からブレーキ状態を判断する
        if max(msg.data) > LINE:
            brake.data = True
        else:
            brake.data = False

        # メッセージをパブリッシュ
        self.publisher.publish(brake)


def main():
    rclpy.init()

    node = DistanceAssessment()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if "__name__" == "__main__":
    main()
