#!/usr/bin/env python3
# Cartographer 2D SLAM 启动文件 - 适配 Livox MID360

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 配置参数
    # 注意：本仓库 cartographer_ros 为源码 vendored（lua 已归其 configuration_files/），但该上游是 catkin(ROS1) 构建，
    # 无法在本机(colcon/humble)编译覆盖 apt → 目录已加 COLCON_IGNORE。
    # 运行时走 apt 版 cartographer_ros，lua 目录需显式传参（见 docs/mapping/ 指南；源码路径示例：
    #   configuration_directory:=<workspace>/src/rm_localization/cartographer_ros/cartographer_ros/configuration_files）
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    use_bag_play = LaunchConfiguration('use_bag_play', default='false')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=PathJoinSubstitution([FindPackageShare('cartographer_ros'), 'configuration_files']),
            description='Full path to directory containing the .lua configuration file'
        ),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer.lua',
            description='Basename of the .lua configuration file'
        ),
        
        # Cartographer 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                ('points2', '/livox/lidar'),  # Livox MID360 点云话题
                ('imu', '/livox/imu'),         # IMU 话题
            ]
        ),
        
        #  occupancy grid 节点（将子图转换为栅格地图）
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05,  # 5cm 分辨率
            }],
            remappings=[
                ('map', '/cartographer_map'),
            ]
        ),
    ])
