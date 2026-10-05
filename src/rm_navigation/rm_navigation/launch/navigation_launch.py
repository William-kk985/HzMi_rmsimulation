# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


# ===================== 代价地图"实时障碍来源"槽位 → 图层开关 =====================
# 全局（默认 stvl = 保持原行为）：
#   stvl : 3D 体素层（STVL，吃 /segmentation/obstacle，带高度带与时间衰减）
#   scan : 2D 障碍层（吃 /scan，与 local_costmap 同源 → "什么算障碍"只在感知域 p2l 决策一处）
#   none : 只用 static + inflation（全局规划看不到实时障碍，靠局部兜底）
# 局部（默认 scan = 保持原行为）：
#   scan      : 只吃 /scan（几何 2D；链路是串行单点）
#   cloud     : 只吃 /segmentation/obstacle（3D 点云直投；不经 p2l，无近距盲区）
#   both      : 双源冗余（scan + cloud；任一路挂掉仍能避障）
#   stvl      : 3D 体素层（STVL，同 global；带 0.5 s 时间衰减，COD 2026 双图同款）
#   stvl_both : 上面两路都给（STVL + scan + cloud；冗余最多、CPU 最贵）
#
# ★ 下面两张表是**唯一真值来源**：槽位值 → {图层: enabled}。launch 由它生成
#   RewrittenYaml 的 enabled 重写；静态校验脚本
#   tools/scripts/regress/local_obstacle_truth_table.py **直接 import 本表 + 本文件的
#   build_param_substitutions()**，再套真实 params 文件跑一遍真 RewrittenYaml
#   ⇒ "文档里的表 / launch 里的表 / 节点最终收到的参数" 三者不会各说各话。
#   为什么不直接按槽位重写 plugins 列表：nav2_common 的 RewrittenYaml 只替换**文件里已存在
#   的叶子键**（rewritten_yaml.py:110-122 的 pathify 只收叶子），且 convert() 只会把字符串转
#   bool/数字（:175-190）—— 字符串值到不了节点（plugins 要的是 string[]，会 type error）。
#   ⇒ 三层全部写进 plugins，只用 enabled 切（= 本仓库 global 既有机制）。
LOCAL_OBSTACLE_LAYER_TABLE = {
    #             obstacle_layer(/scan)   obstacle_cloud_layer(点云直投)  stvl_layer(3D 体素)
    'scan':      {'obstacle_layer': True,  'obstacle_cloud_layer': False, 'stvl_layer': False},
    'cloud':     {'obstacle_layer': False, 'obstacle_cloud_layer': True,  'stvl_layer': False},
    'both':      {'obstacle_layer': True,  'obstacle_cloud_layer': True,  'stvl_layer': False},
    'stvl':      {'obstacle_layer': False, 'obstacle_cloud_layer': False, 'stvl_layer': True},
    'stvl_both': {'obstacle_layer': True,  'obstacle_cloud_layer': True,  'stvl_layer': True},
}
GLOBAL_OBSTACLE_LAYER_TABLE = {
    'stvl': {'obstacle_layer': False, 'stvl_layer': True},
    'scan': {'obstacle_layer': True,  'stvl_layer': False},
    'none': {'obstacle_layer': False, 'stvl_layer': False},
}

# 两张代价地图在 params 文件里的前缀（RewrittenYaml 的键路径用）
GC_PREFIX = 'global_costmap.global_costmap.ros__parameters.'
LC_PREFIX = 'local_costmap.local_costmap.ros__parameters.'


def _enabled_expr(arg_name, table, layer):
    """生成 "<arg> == 'v1' or <arg> == 'v2'" 形式的 PythonExpression（True = 该图层开）。

    @param arg_name launch 参数名（'global_obstacle' / 'local_obstacle'）
    @param table    上面两张真值表之一
    @param layer    图层名（必须是该表里出现过的键）
    """
    parts = []
    for value, flags in table.items():
        if flags[layer]:
            if parts:
                parts.append(' or ')
            parts += ["'", LaunchConfiguration(arg_name), "' == '", value, "'"]
    # 一个值都不开 ⇒ 常量 False（例如 global_obstacle:=none 时的两层）
    return PythonExpression(parts if parts else ['False'])


def build_param_substitutions(use_sim_time, autostart):
    """构造 RewrittenYaml 的 param_rewrites（两张真值表 → 五个 enabled 开关）。

    单独抽成模块级函数（而不是塞在 generate_launch_description 里）是为了让静态校验脚本能
    **复用同一份逻辑**算出"每个槽位值下节点真正收到的参数"，而不是在文档里再抄一份表。
    """
    return {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        GC_PREFIX + 'obstacle_layer.enabled':
            _enabled_expr('global_obstacle', GLOBAL_OBSTACLE_LAYER_TABLE, 'obstacle_layer'),
        GC_PREFIX + 'stvl_layer.enabled':
            _enabled_expr('global_obstacle', GLOBAL_OBSTACLE_LAYER_TABLE, 'stvl_layer'),
        LC_PREFIX + 'obstacle_layer.enabled':
            _enabled_expr('local_obstacle', LOCAL_OBSTACLE_LAYER_TABLE, 'obstacle_layer'),
        LC_PREFIX + 'obstacle_cloud_layer.enabled':
            _enabled_expr('local_obstacle', LOCAL_OBSTACLE_LAYER_TABLE, 'obstacle_cloud_layer'),
        # ★ 2026-10-05 新增：局部 STVL（local_obstacle:=stvl | stvl_both）
        LC_PREFIX + 'stvl_layer.enabled':
            _enabled_expr('local_obstacle', LOCAL_OBSTACLE_LAYER_TABLE, 'stvl_layer'),
    }


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('rm_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    # ★ 2026-10-05 params 分层（见 docs/cod_nav_2026_migration_worksheet.md §J）：
    #   params_file = 公共段（base）；params_file_planner / params_file_controller = 槽位文件
    #   （planner_server / controller_server 段）。三份按 base → planner → controller 顺序给节点，
    #   ROS 2 按参数逐键叠加（同名后写覆盖；后一份文件里没写的键仍来自前一份）。
    #   独立使用本 launch 且只给 params_file 时，两个槽位默认 = params_file（重复叠加同一份文件幂等）。
    params_files = [params_file,
                    LaunchConfiguration('params_file_planner'),
                    LaunchConfiguration('params_file_controller')]
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # 代价地图"实时障碍来源"槽位 → 图层 enabled 开关（唯一真值表见模块顶部
    # LOCAL_OBSTACLE_LAYER_TABLE / GLOBAL_OBSTACLE_LAYER_TABLE）
    param_substitutions = build_param_substitutions(use_sim_time, autostart)

    # 每份文件各套一层同样的 RewrittenYaml（use_sim_time / 图层开关等 rewrite 对三份都生效）
    configured_params = [
        RewrittenYaml(
            source_file=f,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
        for f in params_files]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_global_obstacle_cmd = DeclareLaunchArgument(
        'global_obstacle',
        default_value='stvl',
        description='全局代价地图实时障碍来源: stvl(3D体素,默认) | scan(2D /scan,与 local 同源) | none'
    )

    declare_local_obstacle_cmd = DeclareLaunchArgument(
        'local_obstacle',
        default_value='scan',
        choices=['scan', 'cloud', 'both', 'stvl', 'stvl_both'],
        description='局部代价地图障碍来源: '
                    'scan(/scan,默认=原行为) | cloud(3D点云直投,无近距盲区) | both(scan+cloud双源冗余) | '
                    'stvl(3D体素层,吃/segmentation/obstacle,带时间衰减;COD 2026 双图同款) | '
                    'stvl_both(stvl + scan + cloud,冗余最多也最贵)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # ★ 2026-10-05 params 分层：另两份槽位文件（默认 = params_file ⇒ 单文件用法行为不变）
    declare_params_file_planner_cmd = DeclareLaunchArgument(
        'params_file_planner',
        default_value=params_file,
        description='第二份参数文件（planner 槽：planner_server 段）；默认 = params_file')

    declare_params_file_controller_cmd = DeclareLaunchArgument(
        'params_file_controller',
        default_value=params_file,
        description='第三份参数文件（controller 槽：controller_server 段）；默认 = params_file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[*configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[*configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[*configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[*configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[*configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[*configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[*configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_global_obstacle_cmd)
    ld.add_action(declare_local_obstacle_cmd)
    ld.add_action(declare_params_file_cmd)
    # 槽位文件入参必须在 params_file 之后声明（默认值引用 LaunchConfiguration('params_file')）
    ld.add_action(declare_params_file_planner_cmd)
    ld.add_action(declare_params_file_controller_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
