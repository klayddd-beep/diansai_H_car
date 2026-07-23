"""Manual-only launch entry for the optional drone video receiver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fire_video_receiver_pkg")
    params = os.path.join(package_share, "config", "video_receiver.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "display",
            default_value="true",
            description="Open the optional OpenCV viewer window.",
        ),
        Node(
            package="fire_video_receiver_pkg",
            executable="fire_video_receiver",
            name="fire_video_receiver",
            output="screen",
            parameters=[
                params,
                {"display": LaunchConfiguration("display")},
            ],
        ),
    ])
