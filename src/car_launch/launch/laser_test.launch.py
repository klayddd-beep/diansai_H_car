"""Only test the PB0 laser output; no lidar, chassis, network, or mission nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    duration = LaunchConfiguration("duration")

    return LaunchDescription([
        DeclareLaunchArgument(
            "duration",
            default_value="2.0",
            description="Seconds to turn the laser on before automatic shutoff.",
        ),
        Node(
            package="follower_pkg",
            executable="laser_gpio_driver",
            name="laser_gpio_test",
            parameters=[{
                "mock_mode": False,
                "gpio_number": 40,
                "active_low": True,
                "gpio_command": "/usr/bin/gpio",
                "startup_on": True,
                "max_on_s": ParameterValue(duration, value_type=float),
            }],
            output="screen",
        ),
    ])
