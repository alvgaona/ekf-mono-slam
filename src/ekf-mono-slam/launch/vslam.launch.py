from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_dir", default_value="./datasets/desk_translation/"
            ),
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
                arguments=["--ros-args", "--log-level", "info"],
            ),
            Node(
                package="ekf_mono_slam",
                executable="feature_detector",
                name="feature_detector",
                namespace="slam",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ]
    )
