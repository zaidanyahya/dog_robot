import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

# Constants
NODE_NAME = "color_input"
JOY = "/joy"
COLOR_TOPIC = "color_topic"


class ColorInput(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        self.subscription = self.create_subscription(
            Joy, JOY, self.listener_callback, 10
        )

        self.publisher = self.create_publisher(Int32, COLOR_TOPIC, 10)
        self.color = Int32()

    def listener_callback(self, msg: Joy):
        d = msg.axes[4]
        e = msg.axes[5]
        self.logger(f"d:{d}, e:{e}")

        color = 0
        if d == 1:
            color = 1
        elif d == -1:
            color = 2
        elif e == 1:
            color = 3
        elif e == -1:
            color = 4
        self.logger(f"c:{color}")

        self.color.data = color
        self.publisher.publish(self.color)


def main():
    rclpy.init()

    node = ColorInput()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
