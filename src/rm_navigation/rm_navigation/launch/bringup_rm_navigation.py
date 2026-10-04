import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('rm_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    # ★ 2026-10-05 params 分层（见 docs/cod_nav_2026_migration_worksheet.md §J）：
    #   params_file = 公共段（base）；另两份是同一次传参的槽位文件（planner_server / controller_server 段）。
    #   三份按 base → planner → controller 的顺序给节点（ROS 2 按参数逐键叠加，后写覆盖）。
    #   单独用本 launch 且只给 params_file 时，两个槽位默认 = params_file
    #   （同一份文件重复叠加是幂等的 ⇒ 单文件老用法行为不变）。
    params_files = [params_file,
                    LaunchConfiguration('params_file_planner'),
                    LaunchConfiguration('params_file_controller')]
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    # 每份文件各套一层同样的 RewrittenYaml（use_sim_time / yaml_filename 等 rewrite 对三份都生效）
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

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='True',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value= os.path.join(bringup_dir,'map', 'RMUL.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

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
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_global_obstacle_cmd = DeclareLaunchArgument(
        'global_obstacle',
        default_value='stvl',
        description='全局代价地图实时障碍来源: stvl | scan | none（透传给 navigation_launch）'
    )

    declare_local_obstacle_cmd = DeclareLaunchArgument(
        'local_obstacle',
        default_value='scan',
        description='局部代价地图障碍来源: scan（默认，原行为）| cloud（3D 点云直投）| both（双源冗余）'
    )

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_mt',
            parameters=[*configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              # ★ 2026-10-05 params 分层：槽位文件一并透传给 navigation_launch
                              'params_file_planner': LaunchConfiguration('params_file_planner'),
                              'params_file_controller': LaunchConfiguration('params_file_controller'),
                              'global_obstacle': LaunchConfiguration('global_obstacle'),
                              'local_obstacle': LaunchConfiguration('local_obstacle'),
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
            condition=IfCondition(use_nav_rviz)
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    # 槽位文件入参必须在 params_file 之后声明（默认值引用 LaunchConfiguration('params_file')）
    ld.add_action(declare_params_file_planner_cmd)
    ld.add_action(declare_params_file_controller_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_global_obstacle_cmd)
    ld.add_action(declare_local_obstacle_cmd)
    ld.add_action(declare_nav_rviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
