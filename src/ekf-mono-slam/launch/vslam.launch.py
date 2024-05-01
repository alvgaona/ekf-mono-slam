from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_dir", default_value="./resources/desk_translation/"
            ),
            Node(
                package="ekf-mono-slam",
                executable="file_sequence_image",
                name="file_sequence_image",
                output="screen",
                parameters=[
                    {"image_dir": LaunchConfiguration("image_dir")},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
