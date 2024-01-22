import launch
from launch import LaunchDescription
from launch.actions import LogInfo

from dog_robot.settings import PACKAGE_NAME

name = " ".join(PACKAGE_NAME.split("_")).capitalize()


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(msg=f"{name} starting...."),
        ]
    )
