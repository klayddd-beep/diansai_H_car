"""car 端建图/定位启动:雷达 → robot_state_publisher → cartographer。

结构与 fly_car 的 fly_carto.launch.py 一致;lua 用 car.lua。
雷达串口/参数在 bluesea2 的 params/uart_lidar.yaml,按车的雷达实配修改。
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('bluesea2').find('bluesea2'), 'launch', 'uart_lidar.launch')
        )
    )

    pkg_share = FindPackageShare('car_carto_pkg').find('car_carto_pkg')
    urdf_file = os.path.join(pkg_share, 'urdf', 'car.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False}],
        output='screen'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', pkg_share + '/configuration_files',
            '-configuration_basename', 'car.lua'],
        remappings=[
            ('scan', 'scan')],
        output='screen'
    )

    return LaunchDescription([
        TimerAction(
            period=0.0,
            actions=[lidar_launch]
        ),
        TimerAction(
            period=1.0,
            actions=[robot_state_publisher_node]
        ),
        TimerAction(
            period=10.0,
            actions=[cartographer_node]
        ),
    ])
