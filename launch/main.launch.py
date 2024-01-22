from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir

from dog_robot.settings import PACKAGE_NAME


def generate_launch_description():
    name = " ".join(PACKAGE_NAME.split("_")).capitalize()

    movement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "movement.launch.py"])
        )
    )

    return LaunchDescription([LogInfo(msg=f"{name} starting...."), movement_launch])
