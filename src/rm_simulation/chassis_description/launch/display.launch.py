"""chassis_description 的 ROS 2 版 display.launch —— 逐项对应原 ROS 1 的 launch/display.launch
（joint_state_publisher_gui + robot_state_publisher + rviz2），用来**单独看原模型**。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('chassis_description')
    urdf = os.path.join(share, 'urdf', 'chassis.urdf')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': open(urdf, encoding='utf-8').read()}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'config', 'display.rviz')]
             if os.path.exists(os.path.join(share, 'config', 'display.rviz')) else []),
    ])
