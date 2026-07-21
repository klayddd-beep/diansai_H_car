from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    chassis_timeout_ms = LaunchConfiguration("chassis_timeout_ms")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyS6",
                description="UART device path connected to the chassis controller.",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="UART baud rate used by the chassis controller.",
            ),
            DeclareLaunchArgument(
                "chassis_timeout_ms",
                default_value="0",
                description="Chassis-side comm timeout ($SET,TIMEOUT). "
                "0 disables; enable (e.g. 500) for autonomous cmd_vel tasks.",
            ),
            Node(
                package="orangepi_to_car",
                executable="orangepi_to_carv2",
                name="orangepi_to_carv2",
                output="screen",
                arguments=[
                    "--port", port,
                    "--baud", baud,
                    "--chassis-timeout-ms", chassis_timeout_ms,
                ],
            ),
        ]
    )
