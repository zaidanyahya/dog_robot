# ROS2 Libraries
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# Package and Node Names
from dog_robot.settings import PACKAGE_NAME as package_name
from dog_robot.color.color_input import NODE_NAME as color_input

LAUNCH_NAME = "color"


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    color_input_node = Node(
        package=package_name,
        namespace=package_name,
        executable=color_input,
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"{LAUNCH_NAME} module starting..."),
            joy_node,
            color_input_node,
        ]
    )
