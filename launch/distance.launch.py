# ROS2 Libraries
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# Package and Node Names
from dog_robot.settings import PACKAGE_NAME as package_name
from dog_robot.distance.distance_assessment import NODE_NAME as distance_assessment
from dog_robot.distance.distance_input import NODE_NAME as distance_input

LAUNCH_NAME = "distance"


def generate_launch_description():
    distance_assessment_node = Node(
        package=package_name,
        namespace=package_name,
        executable=distance_assessment,
    )

    distance_input_node = Node(
        package=package_name, namespace=package_name, executable=distance_input
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"{LAUNCH_NAME} module starting..."),
            distance_assessment_node,
            distance_input_node,
        ]
    )
