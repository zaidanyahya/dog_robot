from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node

from dog_robot.settings import PACKAGE_NAME
from dog_robot.sound.random_sound import NODE_NAME as random_sound


def generate_launch_description():
    name = " ".join(PACKAGE_NAME.split("_")).capitalize()

    color_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "color.launch.py"])
        )
    )

    distance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "distance.launch.py"])
        )
    )

    movement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "movement.launch.py"])
        )
    )

    random_sound_node = Node(
        package=PACKAGE_NAME, namespace=PACKAGE_NAME, executable=random_sound
    )

    target_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "target_tracker.launch.py"])
        )
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"{name} starting...."),
            color_launch,
            distance_launch,
            movement_launch,
            random_sound_node,
            target_tracker_launch,
        ]
    )
