import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

# =============================================================================
# localization:=beluga —— beluga_amcl 重定位槽的启动文件（2026-10-05 新增）
#
#   本文件是 localization_amcl_launch.py 的**平行实现**（同一形态，只换实现方）：
#     ① 与 amcl 槽一样，map_server 与定位节点由【同一个】lifecycle_manager 管
#        （node_names = ['map_server','amcl']）⇒ map_server 照常在跑，/map 有且只有一个发布者；
#     ② 定位节点名固定为 amcl（beluga_amcl 的节点基类硬编码名字 "amcl"；
#        上游 own example 也按 name='amcl' 起，见 beluga_example/launch/utils/localization_launch.py）；
#     ③ 只发 map→odom（tf_broadcast=true 在参数文件里），绝不碰 odom→base_link。
#
#   ⚠️ 与 nav2 的 amcl 槽的三处**真实差异**（详见 docs/localization_slots.md §1.2）：
#     · 参数文件是 beluga 专用兄弟文件 ../params/nav2_params_sim_beluga.yaml
#       （参数集不是超集关系；base 里的 amcl 段仍留给 localization:=amcl）；
#     · beluga **没有** initial_pose.z ⇒ 本文件不声明/不注入 z（amcl 那份有）；
#     · 位姿话题是 `pose`（≈ /pose）而不是 `amcl_pose`，且**不是参数**（要改名只能 remap）。
#
#   ⚠️ beluga_amcl 是外部依赖（apt: ros-humble-beluga-amcl 或源码构建）。
#     本文件**不在生成期**调用 get_package_share_directory('beluga_amcl')
#     ⇒ 没装 beluga 也不会把其它槽位/其它 mode 的 launch 一起带崩；
#     只有真的选了 localization:=beluga 时，下面的 OpaqueFunction 会**显式报错**并给出安装命令。
# =============================================================================


def _check_beluga_amcl_available(context):
    """选了 localization:=beluga 才执行的预检：beluga_amcl 不在 AMENT_PREFIX_PATH 就明确报错。

    动机：Node(package='beluga_amcl') 的报错发生在 launch 已经起了 Gazebo/LIO/map_server 之后，
    排查者看到的是"进程起不来"而不是"少装了一个包"⇒ 这里提前失败，并直接给出两条安装路线。
    """
    try:
        prefix = get_package_prefix('beluga_amcl')
    except PackageNotFoundError:
        raise RuntimeError(
            "[localization:=beluga] 找不到包 beluga_amcl。请任选一条路线：\n"
            "  (A) 二进制（推荐，Humble 有官方包）：sudo apt install ros-humble-beluga-amcl\n"
            "  (B) 源码：把 Ekumen-OS/beluga 放到 colcon 会构建的位置（例如 src/rm_localization/beluga，"
            "注意 third_party/ 有 COLCON_IGNORE，colcon 不会构建那里），依赖 libeigen3-dev / libhdf5-dev / "
            "librange-v3-dev / libtbb-dev / ros-humble-sophus，然后 colcon build --packages-up-to beluga_amcl。\n"
            "  详见 docs/localization_slots.md §1.2。")
    # 只是为了让"检查过了"这件事在日志里可见（不改变任何行为）
    print(f'[localization:=beluga] beluga_amcl found at: {prefix}')
    return []


def generate_launch_description():
    # 与 localization_amcl_launch.py 一致的默认值来源（仅用于 params_file 的兜底默认）
    bringup_dir = get_package_share_directory('nav2_bringup')
    own_params_dir = os.path.join(get_package_share_directory('rm_navigation'), 'params')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    # ★ 与 amcl 槽同一套 params 分层约定（见 docs/cod_nav_2026_migration_worksheet.md §J）：
    #   params_file = 公共段（base，map_server 段在这里）；另两份是 nav2 的槽位文件。
    #   本 launch 里只有 map_server 用得上 base；另外两份仅为与 map_server_launch.py 传参保持一致。
    params_files = [params_file,
                    LaunchConfiguration('params_file_planner'),
                    LaunchConfiguration('params_file_controller')]
    # ★ beluga 定位节点自己的参数文件（独立一份，见该文件头部注释里的参数集差异说明）
    beluga_params_file = LaunchConfiguration('beluga_params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    # 与 amcl 槽同一条经验（localization_amcl_launch.py 里的注释）：
    # map_server 与 amcl 必须由【同一个】lifecycle_manager 管理，否则会出两个同名
    # lifecycle_manager_localization 抢同一批节点 ⇒ map_server 无法激活 ⇒ 定位节点永远等地图。
    lifecycle_nodes = ['map_server', 'amcl']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # use_sim_time / 地图 / 初值（全路径写法，避免 'x'/'y' 这类短叶子名误伤全局同名参数）。
    # ⚠️ 故意**没有** initial_pose.z：beluga 的 2D 节点不声明该键（nav2_amcl 声明了）。
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'amcl.ros__parameters.initial_pose.x': initial_pose_x,
        'amcl.ros__parameters.initial_pose.y': initial_pose_y,
        'amcl.ros__parameters.initial_pose.yaw': initial_pose_yaw}

    # map_server 用 base（+ 槽位文件，与 map_server_launch.py 传参一致）
    configured_params = [
        ParameterFile(
            RewrittenYaml(
                source_file=f,
                root_key=namespace,
                param_rewrites=param_substitutions,
                convert_types=True),
            allow_substs=True)
        for f in params_files]
    # beluga amcl 节点用 beluga 专用文件
    configured_beluga_params = ParameterFile(
        RewrittenYaml(
            source_file=beluga_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='公共段参数文件（map_server 段在这里）')

    declare_params_file_planner_cmd = DeclareLaunchArgument(
        'params_file_planner',
        default_value=params_file,
        description='第二份参数文件（planner 槽）；默认 = params_file（本 launch 未使用）')

    declare_params_file_controller_cmd = DeclareLaunchArgument(
        'params_file_controller',
        default_value=params_file,
        description='第三份参数文件（controller 槽）；默认 = params_file（本 launch 未使用）')

    declare_beluga_params_file_cmd = DeclareLaunchArgument(
        'beluga_params_file',
        default_value=os.path.join(own_params_dir, 'nav2_params_sim_beluga.yaml'),
        description='beluga_amcl 定位节点的参数文件（amcl 段，见 docs/localization_slots.md §1.2）')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True（与 amcl 槽一样，默认关闭）')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of container that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='用于 map_server 的栅格地图 yaml 路径（由 bringup 传入）')

    # beluga 的初值是 map 系 (x, y, yaw)；没有 z
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0',
        description='beluga AMCL 初始位姿 x（map 系，米）')
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0',
        description='beluga AMCL 初始位姿 y（map 系，米）')
    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0',
        description='beluga AMCL 初始位姿 yaw（map 系，弧度）')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[*configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # beluga_amcl 的 amcl_node 是 **lifecycle** 节点（rclcpp_lifecycle::LifecycleNode，
            # 且带 bondcpp bond）⇒ 与 nav2 amcl 一样由 lifecycle_manager autostart 拉起。
            # 节点名必须叫 amcl（上游基类硬编码名字）；参数文件只给 beluga 专用那一份。
            Node(
                package='beluga_amcl',
                executable='amcl_node',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_beluga_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[*configured_params],
                remappings=remappings),
            # 组件插件名核实：beluga_amcl/CMakeLists.txt 的
            # rclcpp_components_register_node(amcl_node_component PLUGIN "beluga_amcl::AmclNode" EXECUTABLE amcl_node)
            ComposableNode(
                package='beluga_amcl',
                plugin='beluga_amcl::AmclNode',
                name='amcl',
                parameters=[configured_beluga_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
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
    ld.add_action(declare_params_file_cmd)
    # 槽位文件入参必须在 params_file 之后声明（默认值引用 LaunchConfiguration('params_file')）
    ld.add_action(declare_params_file_planner_cmd)
    ld.add_action(declare_params_file_controller_cmd)
    ld.add_action(declare_beluga_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)

    # 预检：beluga_amcl 是否在环境里（选了这个槽才执行）
    ld.add_action(OpaqueFunction(function=_check_beluga_amcl_available))

    # Add the actions to launch all of the localization nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
