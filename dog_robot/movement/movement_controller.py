# Standard Libraries
import math
import time

# ROS2 Libraries
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

# Constants
NODE_NAME = "movement_controller"
BRAKE = "brake_topic"
DIRECTION = "direction_topic"
CMD_VEL = "/cmd_vel"
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.1 * math.pi
SEARCH_ANGULAR_VEL = 0.5 * math.pi
RANDOM_ROT_ANGULAR_VEL = math.pi


# Raspimouse movement controller Node
class MovementController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # Subcriptions
        self.distance_as_sub = self.create_subscription(
            Bool, BRAKE, self.brake_callback, 10
        )
        self.direction_sub = self.create_subscription(
            Float64MultiArray, DIRECTION, self.direction_callback, 10
        )

        # Raspimouse Movement Instruction Publisher
        self.motion_publisher = self.create_publisher(Twist, CMD_VEL, 1)

        # Internal State
        self.brake = True
        self.forward_direction = 0
        self.angular_direction = 0
        self.search_called = False
        self.search_time = time.time()
        self.search_duration = 1000
        self.start = 0

    # brake_topic callback
    def brake_callback(self, msg: Bool):
        self.brake = bool(msg.data)

    # direction_topic callback
    def direction_callback(self, msg: Float64MultiArray):
        self.start = msg.data[0]
        self.forward_direction = msg.data[1]
        self.angular_direction = msg.data[2]
        self.raspimouse_control()

    # Control logic for Raspimouse motor
    def raspimouse_control(self):
        if self.brake:
            self.stop()
            return

        if self.forward_direction < 0.5 and self.start > 0.5:
            self.search()
        else:
            self.move()

    # Initiate search behavior
    def search(self):
        self.rotate(SEARCH_ANGULAR_VEL)

    # Random rotation behavior
    def random_rotation(self):
        self.rotate(RANDOM_ROT_ANGULAR_VEL)

    # Stop Raspimouse movement
    def stop(self):
        msg = Twist()
        self.motion_publisher.publish(msg)

    # Rotate Raspimouse at a given angular velocity
    def rotate(self, velocity: float):
        msg = Twist()
        msg.angular.z = velocity
        self.motion_publisher.publish(msg)

    # Move Raspimouse forward with a certain linear and angular speed
    def move(self):
        msg = Twist()
        msg.linear.x = LINEAR_SPEED
        msg.angular.z = ANGULAR_SPEED * self.angular_direction
        self.motion_publisher.publish(msg)


def main():
    rclpy.init()

    node = MovementController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
