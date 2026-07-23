"""消防车正式入口：定位、差速底盘、火情 UDP、任务状态机和激光 GPIO 抽象。"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def include(package, filename, arguments=None):
    share = FindPackageShare(package=package).find(package)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", filename)), launch_arguments=arguments.items() if arguments else None)

def generate_launch_description():
    share = FindPackageShare(package="follower_pkg").find("follower_pkg")
    params = os.path.join(share, "config", "fire_params.yaml")
    chassis_port = LaunchConfiguration("chassis_port")
    chassis_baud = LaunchConfiguration("chassis_baud")
    return LaunchDescription([
        DeclareLaunchArgument(
            "chassis_port",
            default_value="/dev/ttyS6",
            description="UART device connected to the SR5E1E3 chassis.",
        ),
        DeclareLaunchArgument(
            "chassis_baud",
            default_value="115200",
            description="SR5E1E3 UART baud rate.",
        ),
        include("car_carto_pkg", "car_carto.launch.py"),
        TimerAction(period=3.0, actions=[
            include(
                "orangepi_to_car",
                "orangepi_to_carv2.launch.py",
                {
                    "port": chassis_port,
                    "baud": chassis_baud,
                    "chassis_timeout_ms": "500",
                },
            ),
            Node(package="follower_pkg", executable="diff_drive_controller", parameters=[os.path.join(share, "config", "follower_params.yaml")], output="screen"),
            Node(package="follower_pkg", executable="fire_event_bridge", parameters=[params], output="screen"),
            Node(package="follower_pkg", executable="fire_link_bridge", parameters=[params], output="screen"),
            Node(package="follower_pkg", executable="fire_vision_node.py", parameters=[params], output="screen"),
            Node(package="follower_pkg", executable="fire_mission_manager", parameters=[params], output="screen"),
            Node(package="follower_pkg", executable="laser_gpio_driver", parameters=[params], output="screen"),
            Node(package="follower_pkg", executable="fire_dashboard.py", parameters=[params], output="screen"),
        ])
    ])
