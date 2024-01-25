# ROS2 Libraries
import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_srvs.srv import SetBool
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState

# Constants
NODE_NAME = "motor_client_async"
GET_STATE = "/raspimouse/get_state"
CHANGE_STATE = "/raspimouse/change_state"
MOTOR_POWER = "/motor_power"


class MotorClientAsnyc(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.logger = self.get_logger()

        # Raspimouse GetState
        self.client_get_state = self.create_client(GetState, GET_STATE)
        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.warn_service_not_available(self.client_get_state)

        # Raspimouse ChangeState
        self.client_change_state = self.create_client(ChangeState, CHANGE_STATE)
        while not self.client_change_state.wait_for_service(timeout_sec=1.0):
            self.warn_service_not_available(self.client_change_state)
        self.activate_raspimouse()

        # Raspimouse Motor Power
        self.client_motor_power = self.create_client(SetBool, MOTOR_POWER)
        while not self.client_motor_power.wait_for_service(timeout_sec=1.0):
            self.warn_service_not_available(self.client_motor_power)

        self.request = SetBool.Request()
        self.motor_power(True)

    def warn_service_not_available(self, client: Client) -> None:
        self.logger.warn(client.srv_name + " service not available, waiting again...")

    def activate_raspimouse(self) -> None:
        self.set_mouse_lifecycle_state(Transition.TRANSITION_CONFIGURE)
        self.set_mouse_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        self.logger.info("Mouse state is " + self.get_mouse_lifecycle_state())

    def set_mouse_lifecycle_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client_change_state.call_async(request)
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        result: ChangeState.Response = future.result()
        return result.success

    def get_mouse_lifecycle_state(self):
        future = self.client_get_state.call_async(GetState.Request())
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        result: GetState.Response = future.result()
        return result.current_state.label

    def motor_power(self, request_data=False):
        self.request.data = request_data
        future = self.client_motor_power.call_async(self.request)
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        result: SetBool.Response = future.result()
        self.logger.info(result.message)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.motor_power(False)
        self.set_mouse_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
        return super().on_deactivate(state)


def main():
    rclpy.init()

    node = MotorClientAsnyc()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if "__name__" == "__main__":
    main()
