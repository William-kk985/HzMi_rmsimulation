#!/usr/bin/env python3
# Cartographer 2D SLAM 启动文件 - 适配 Livox MID360
# 支持两种用法：
#   1) 建图：configuration_basename:=cartographer.lua（默认）
#   2) 纯定位：configuration_basename:=cartographer_localization.lua \
#              load_state_filename:=<xxx.pbstream> load_frozen_state:=true

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _cartographer_nodes(context, *args, **kwargs):
    """按参数动态拼 cartographer_node 的 arguments（load_state_filename 可选）。"""
    config_dir = LaunchConfiguration('configuration_directory').perform(context)
    config_base = LaunchConfiguration('configuration_basename').perform(context)
    load_state = LaunchConfiguration('load_state_filename').perform(context)
    frozen = LaunchConfiguration('load_frozen_state').perform(context)

    arguments = ['-configuration_directory', config_dir,
                 '-configuration_basename', config_base]
    if load_state.strip():                       # 纯定位模式：加载已有 pbstream
        arguments += ['-load_state_filename', load_state.strip()]
        if str(frozen).lower() in ('true', '1'):
            arguments += ['-load_frozen_state', 'true']

    occupancy_grid_topic = LaunchConfiguration('occupancy_grid_topic')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=arguments,
        remappings=[
            # 2D 激光路线（当前默认，见 configuration_files/cartographer.lua）：
            #   /scan 由感知域 linefit(去地面) + pointcloud_to_laserscan(高度带) 产出
            ('scan', '/scan'),
            # 备选点云路线：cartographer_ros **只支持 sensor_msgs/PointCloud2**，
            # 绝不能 remap 到 /livox/lidar（那是 livox_ros_driver2/CustomMsg，类型不匹配 -> 收不到数据）
            # ('points2', '/livox/lidar/pointcloud'),
            ('imu', '/livox/imu'),
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,          # 5cm 分辨率
        }],
        # 栅格话题：**默认发到标准话题 /map**，这样 nav2 的 static_layer、map_saver_cli、
        # RViz 的 Map 显示项都能直接吃到它（原先硬 remap 到 /cartographer_map，
        # 结果是"边建边导 mapper:=cartographer 时 costmap 没有静态图"、
        # "建完图 map_saver_cli 存不到东西"两个静默失效）。
        # 例外：若纯定位时另有 map_server 在发先验 /map，会形成两个发布者 → 那时传
        # occupancy_grid_topic:=/cartographer_map 规避（本流程暂不可用，缺 pbstream）。
        remappings=[('map', occupancy_grid_topic)]
    )

    return [cartographer_node, occupancy_grid_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=PathJoinSubstitution([FindPackageShare('cartographer_ros'), 'configuration_files']),
            description='Full path to directory containing the .lua configuration file'
        ),
        DeclareLaunchArgument(
            'occupancy_grid_topic',
            default_value='map',
            description='cartographer 栅格输出话题。默认 map（建图/边建边导给 nav2 与 map_saver 用）；'
                        '若纯定位时另有 map_server 发先验 /map，传 /cartographer_map 避免双发布者'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer.lua',
            description='Basename of the .lua configuration file (建图: cartographer.lua；纯定位: cartographer_localization.lua)'
        ),
        DeclareLaunchArgument(
            'load_state_filename',
            default_value='',
            description='纯定位用：已有 .pbstream 的完整路径（留空=纯建图模式）'
        ),
        DeclareLaunchArgument(
            'load_frozen_state',
            default_value='true',
            description='纯定位用：是否冻结已加载的状态（配合 load_state_filename）'
        ),
        OpaqueFunction(function=_cartographer_nodes),
    ])
