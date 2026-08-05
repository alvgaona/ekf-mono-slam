import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ekf_mono_slam")
    default_config = os.path.join(pkg_share, "config", "ekf.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_dir", default_value="./datasets/desk_translation/"
            ),
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="ekf_mono_slam",
                executable="file_sequence_image",
                name="file_sequence_image",
                namespace="slam",
                output="screen",
                parameters=[
                    {"image_dir": LaunchConfiguration("image_dir")},
                ],
                arguments=["--ros-args", "--log-level", "info"],
            ),
            Node(
                package="ekf_mono_slam",
                executable="ekf",
                name="ekf",
                namespace="slam",
                output="screen",
                parameters=[LaunchConfiguration("config")],
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ]
    )
