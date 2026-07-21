"""雷达定位闭环跑车测试(不依赖飞车):建图定位 + 航点测试发布 + 差速控制 + 底盘桥。

链路:
  雷达 /scan -> car_carto(Cartographer) -> TF map<-laser_link
  waypoint_test_publisher -> /target_position -> diff_drive_controller -> /cmd_vel
  -> orangepi_to_carv2($VW 流) -> SR5E1E3 底盘

这是消防任务之外的底层回归测试：改用 waypoint_test_publisher 自动喂航点，
用于验证雷达定位、差速控制和底盘桥。航点/容差见 follower_pkg/config/follower_params.yaml。

底盘桥带 chassis_timeout_ms=500(底盘侧通信超时兜底)。
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include_launch(package_name, filename, launch_arguments=None):
    package_share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", filename)),
        launch_arguments=launch_arguments.items() if launch_arguments else None,
    )


def generate_launch_description():
    follower_share = FindPackageShare(package="follower_pkg").find("follower_pkg")
    params_file = os.path.join(follower_share, "config", "follower_params.yaml")

    car_carto_launch = _include_launch("car_carto_pkg", "car_carto.launch.py")
    chassis_launch = _include_launch(
        "orangepi_to_car", "orangepi_to_carv2.launch.py",
        launch_arguments={"chassis_timeout_ms": "500"},
    )

    diff_drive_node = Node(
        package="follower_pkg",
        executable="diff_drive_controller",
        parameters=[params_file],
        output="screen",
    )
    waypoint_test_node = Node(
        package="follower_pkg",
        executable="waypoint_test_publisher",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([
        car_carto_launch,
        # 等 carto/TF 起来再拉控制链;waypoint_test_publisher 自身还有 start_delay_s 兜底
        TimerAction(
            period=3.0,
            actions=[
                chassis_launch,
                diff_drive_node,
                waypoint_test_node,
            ],
        ),
    ])
