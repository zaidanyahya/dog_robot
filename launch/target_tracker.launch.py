# ROS2 Libraries
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# Package and Node Names
from dog_robot.settings import PACKAGE_NAME as package_name
from dog_robot.target_tracker.image_capture import NODE_NAME as image_capture
from dog_robot.target_tracker.target_monitor import NODE_NAME as target_monitor
from dog_robot.target_tracker.target_recognition import NODE_NAME as target_recognition

LAUNCH_NAME = "target_tracker"


def generate_launch_description():
    image_capture_node = Node(
        package=package_name,
        namespace=package_name,
        executable=image_capture,
    )

    target_monitor_node = Node(
        package=package_name,
        namespace=package_name,
        executable=target_monitor,
    )

    target_recognition_node = Node(
        package=package_name,
        namespace=package_name,
        executable=target_recognition,
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"{LAUNCH_NAME} module starting..."),
            image_capture_node,
            target_monitor_node,
            target_recognition_node,
        ]
    )
