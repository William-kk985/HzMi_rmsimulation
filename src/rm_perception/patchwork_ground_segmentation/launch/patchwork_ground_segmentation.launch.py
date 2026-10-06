"""单独起 patchwork 地面分割节点（不经 bringup_sim.launch.py）。

正常用法是走槽位：
    ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping ground:=patchwork
本文件只用于「只想起分割器、不要 Gazebo/LIO」的离线调试（例如对着 ros2 bag play 重放）。

注意：本文件**不含** pointcloud_to_laserscan —— 槽位模式下它是常开节点，由 bringup 负责；
本文件只负责 /segmentation/ground + /segmentation/obstacle。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('patchwork_ground_segmentation')
    default_params = os.path.join(pkg_share, 'config', 'ground_segmentation_sim.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    node = Node(
        package='patchwork_ground_segmentation',
        executable='patchwork_ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='参数文件（默认 = 本包 config/ground_segmentation_sim.yaml）'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        node,
    ])
