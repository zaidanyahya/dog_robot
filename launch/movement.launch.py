# ROS2 Libraries
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# Package and Node Names
from dog_robot.settings import PACKAGE_NAME as package_name
from dog_robot.movement.motor_client_async import NODE_NAME as motor_client_async
from dog_robot.movement.movement_controller import NODE_NAME as movement_controller

LAUNCH_NAME = "movement"


def generate_launch_description():
    motor_client_async_node = Node(
        package=package_name,
        namespace=package_name,
        executable=motor_client_async,
    )

    movement_controller_node = Node(
        package=package_name,
        namespace=package_name,
        executable=movement_controller,
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"{LAUNCH_NAME} module starting..."),
            motor_client_async_node,
            movement_controller_node,
        ]
    )
