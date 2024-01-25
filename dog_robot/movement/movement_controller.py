import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

# Constants
NODE_NAME = "movement_controller"
DISTANCE_AS = "distanceAs_topic"
DIRECTION = "direction_topic"
CMD_VEL = "/cmd_vel"
TIMER_PERIOD = 0.03
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 2.0
SEARCH_ANGULAR_SPEED = 2.0
SEARCH_ANGULAR_DIRECTION = 2.0
RANDOM_ROT_ANGULAR_SPEED = 2.0
RANDOM_ROT_ANGULAR_DIRECTION = 2.0


class MovementController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        self.distance_as_sub = self.create_subscription(
            Bool, DISTANCE_AS, self.brake_callback, 10
        )
        self.direction_sub = self.create_subscription(
            Float64MultiArray, DIRECTION, self.direction_callback, 10
        )

        # Raspimouse Movement Instruction Publisher
        self.motion_publisher = self.create_publisher(Twist, CMD_VEL, 1)

        self.brake = True
        self.forward_backward_direction = 0
        self.angular_direction = 0
        self.search_called = False
        self.search_time = time.time()
        self.search_duration = 10000

        self.create_timer(TIMER_PERIOD, self.raspimouse_control)

    def brake_callback(self, msg: Bool):
        new_brake = bool(msg.data)
        if not self.brake and new_brake:
            self.logger.info(f"new_brake:{new_brake}")
            self.random_rotation()
        self.brake = new_brake

    def direction_callback(self, msg: Float64MultiArray):
        self.forward_backward_direction = msg.data[0]
        self.angular_direction = msg.data[1]

    def raspimouse_control(self):
        if self.brake:
            return

        if self.forward_backward_direction == 0:
            if not self.search_called:
                self.search_called = True
                self.search_time = time.time()
            else:
                if time.time() - self.search_time > self.search_duration:
                    self.stop()
                else:
                    self.search()
        else:
            self.move()

    def search(self):
        self.rotate(SEARCH_ANGULAR_SPEED, SEARCH_ANGULAR_DIRECTION)

    def random_rotation(self):
        self.rotate(RANDOM_ROT_ANGULAR_SPEED, RANDOM_ROT_ANGULAR_DIRECTION)

    def stop(self):
        msg = Twist()
        self.motion_publisher.publish(msg)

    def rotate(self, speed: float, direction: float):
        msg = Twist()
        msg.angular.z = speed * direction
        self.motion_publisher.publish(msg)

    def move(self):
        msg = Twist()
        msg.linear.x = LINEAR_SPEED * self.forward_backward_direction
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
