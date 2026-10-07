#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.actions import SetEnvironmentVariable
from launch.substitution import Substitution

# Enum for world types
class WorldType:
    RMUC = 'RMUC'
    RMUL = 'RMUL'
    RMUL2026 = 'RMUL2026'
    RMUC2026 = 'RMUC2026'

def get_world_config(world_type):
    world_configs = {
        WorldType.RMUC: {
            'x': '6.35',
            'y': '7.6',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUC2024_world/RMUC2024_world.world'
        },
        WorldType.RMUL: {
            'x': '4.3',
            'y': '3.35',
            # 原来写 1.16：场地地面在 z≈0，机器人会悬空 1.1m 落下，
            # 导致 FAST-LIO 在坠落中做重力初始化 → 地图/位姿倾斜。轮半径 0.06 → 落到 0.06，取 0.2 留余量
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUL2024_world/RMUL2024_world.world'
            # 'world_path': 'RMUL2024_world/RMUL2024_world_dynamic_obstacles.world'
        },
        WorldType.RMUL2026: {
            'x': '4.3',
            'y': '3.35',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUL2026_world/RMUL2026_world.world'
        },
        # ★ 2026-10-05 新增：RMUC2026（附件 easystl.stl 生成的 29.15 x 16.05 m 全场，见 docs/worlds.md）
        # 出生点推导（全部脚本与原始栅格在 .tmp_cache/rmuc2026/，结论见 docs/worlds.md §3）：
        #   ① 0.02 m 高度栅格（.tmp_cache/stl_view/heightmap.npz）-> 0.05 m 占用栅格：
        #      底板顶面 z=-1.6413436 m（== world z 0）为唯一可行驶层，0.20/0.30 m 台阶全部算障碍；
        #   ② 在"严格平台面"（相对底板抬升 ≤0.05 m）上算到最近障碍的距离变换，
        #      要求 clearance ≥ robot_radius(0.22) + margin(0.08) = 0.30 m；
        #   ③ 左右两个对称大区各 ~71.4 m²，取 +x 半场里距障碍最远的点 = (10.925, 2.525)，
        #      实测 clearance 2.704 m（左半场镜像点 (-10.925, 0.775) clearance 2.706 m，等价）；
        #      该点 ±0.55 m 邻域内高度起伏 ≤0.004 m（真平）。
        #   z=0.2 与 RMUL2026 同口径：轮半径 0.06 -> 静止时 base_link 在 0.06，留 0.14 m 余量，
        #   既不悬空太久（LIO 会在坠落中做重力初始化）也不会插进地面。
        WorldType.RMUC2026: {
            'x': '10.925',
            'y': '2.525',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUC2026_world/RMUC2026_world.world'
        }
    }
    return world_configs.get(world_type, None)

class _RobotSlotXacro(Substitution):
    """robot:=<模型> 槽位 → 某个包的 urdf/ 下的 xacro 文件名（惰性，包也是惰性查的）。

    ★ 2026-10-07 新增，与 rm_nav_bringup/launch/bringup_sim.launch.py 里的同类**语义一致**
    （那边是本仓的正式入口；这里让"只起 Gazebo+模型、不起感知/LIO"的单独调试也能选模型）。
    默认值（''）返回的文件与改造前**完全相同** ⇒ 默认行为一个字节都不变。
    ⚠️ 本文件不含 linefit/p2l/LIO 节点 ⇒ 这里选 hzmirm **不会**自动切换感知标定与 LIO 杆臂，
    完整链路请走 `ros2 launch rm_nav_bringup bringup_sim.launch.py robot:=hzmirm ...`；
    云台角（turret_yaw_deg/turret_pitch_deg）也只有那边会传，这里取默认 0（= 与上游 URDF 等价）。
    """

    _SLOTS = {
        '': ('hzmi_rm_simulation', 'simulation_waking_robot.xacro'),
        'hzmirm': ('rm_nav_bringup', 'sentry_robot_hzmirm_sim.xacro'),
        # ★ 2026-10-07：用户的哨兵 robot11（真 mesh；碰撞件按 Phase 1 实测换过）。
        #   本文件不带额外 xacro 参数 ⇒ 用模型自己的默认值（livox_tilt_rpy 默认 = roll，
        #   与 launch 的 livox_tilt_axis 默认一致）。
        'robot11': ('rm_nav_bringup', 'sentry_robot_robot11_sim.xacro'),
    }

    def perform(self, context):
        slot = LaunchConfiguration('robot').perform(context).strip()
        if slot not in self._SLOTS:
            raise RuntimeError(
                "[launch] robot:=%r 不是可用的机器人模型槽位：本文件只认 %s"
                '（完整链路见 rm_nav_bringup/launch/bringup_sim.launch.py + docs/robot_models.md）'
                % (slot, ' / '.join("robot:=%s" % (k or "''") for k in sorted(self._SLOTS))))
        package, filename = self._SLOTS[slot]
        return os.path.join(get_package_share_directory(package), 'urdf', filename)

    def describe(self):
        return '%s(slots=%s)' % (type(self).__name__, ','.join(sorted(self._SLOTS)))


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('hzmi_rm_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path（默认槽位 = 本文件原来的那份 xacro，命令字符串与改造前逐字节相同）
    default_robot_description = Command(['xacro ', _RobotSlotXacro()])

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')
    robot_description = LaunchConfiguration('robot_description')

    # Set Gazebo plugin path
    append_enviroment = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(os.path.join(get_package_share_directory('hzmi_rm_simulation'), 'meshes', 'obstacles', 'obstacle_plugin', 'lib'))
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RMUC,
        description='Choose <RMUC>, <RMUL>, <RMUL2026> or <RMUC2026>'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )

    # ★ 2026-10-07：机器人模型槽位（**opt-in**；默认 '' = 本文件原来的模型 ⇒ 行为不变）。
    #   完整链路（感知标定 + LIO 杆臂一起切）请用 rm_nav_bringup/launch/bringup_sim.launch.py。
    declare_robot_cmd = DeclareLaunchArgument(
        'robot',
        default_value='',
        description="机器人模型槽位：'' = simulation_waking_robot.xacro（本文件原来的模型，默认）；"
                    'hzmirm = rm_nav_bringup/urdf/sentry_robot_hzmirm_sim.xacro；'
                    'robot11 = rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro'
                    '（用户的哨兵 robot11：真 mesh 视觉 + 真 inertial + 雷达斜 30° 下俯+4 个 box 碰撞）'
                    '（用户给的哨兵 URDF，雷达在云台头上、离地 ~0.8 m，底盘 0.6x0.6x0.3）。'
                    '⚠️ 本文件不起感知/LIO ⇒ 选 hzmirm 不会自动切 linefit sensor_height 与 '
                    'lio_tf_adapter 杆臂；要完整链路请用 bringup_sim.launch.py robot:=hzmirm。'
                    '详见 docs/robot_models.md',
        choices=['', 'hzmirm', 'robot11']
    )

    # ★ 2026-10-07：Gazebo GUI / 在线模型库开关（**都是 opt-in，默认 = 改造前行为**）。
    #   gui=False 的用途：一条命令无头跑（省 gzclient 的 ~450 MiB 与 1 个 GL 上下文），
    #   也免掉"事后 pkill gzclient"这一步（本仓多个 bench 脚本会 `pkill -9 -x gzclient`，
    #   那是**全机**范围的 —— 会把别人正在看的 GUI 一起杀掉，见
    #   docs/gazebo_gui_troubleshooting.md §"exit code -9 到底是谁杀的"）。
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='True（默认）= 起 gzclient（Gazebo 图形界面，与改造前逐字相同）；'
                    'False = 只起 gzserver（无头；RViz 不受影响，nav_rviz/lio_rviz 照旧）'
    )

    #   为什么需要 gazebo_offline（2026-10-07 实测口径，别照抄"网络慢"这种含糊说法）：
    #   `ModelDatabase` 构造函数就起一个后台线程去拉 http://models.gazebosim.org/ 的清单，
    #   而这条 libcurl **没有设任何超时**。日常它不挡路（清单抓完就算了）；**只有当
    #   GUI 线程也需要模型清单时**（`GetModels()` 抢不到锁 ⇒ 打印
    #   "Waiting for model database update to complete..." 并**同步阻塞**）才会卡住界面。
    #   实测：`robot:=robot11` 的 12 个 `<visual>` 是 `package://robot11/...`，
    #   sdformat 转 SDF 时改写成 `model://robot11/...`；这些 URI 在
    #   `GAZEBO_MODEL_PATH` 上解析不到 ⇒ `SystemPaths::FindFileURI()` **无条件回落**到
    #   `ModelDatabase::GetModelPath(uri, /*forceDownload=*/true)` ⇒ 撞上那把锁。
    #   黑洞代理下实测 **stall ≥ 99.95 s（不设上限）**，用户 19:50 日志实测 **48.03 s**
    #   且是被 Ctrl-C 打断的。⇒ 治本是让 `model://robot11/...` 本地可解析
    #   （docs/gazebo_gui_troubleshooting.md §5.1）；本开关只是"不再等"的兜底。
    declare_gazebo_offline_cmd = DeclareLaunchArgument(
        'gazebo_offline',
        default_value='False',
        description='False（默认，行为不变）= 用 gazebo 自带的在线模型库地址；'
                    'True = 把 GAZEBO_MODEL_DATABASE_URI 指到 http://127.0.0.1:1/ '
                    '（连接立刻被拒）⇒ 后台清单抓取立刻失败，GUI 不再可能卡在"等模型库"。'
                    '⚠️ 它**不会**让车在 Gazebo 里出现（那是 robot11 的 model:// 解析问题）；'
                    '⚠️ 别用空串（gazebo 的 GetURI() 对空串有越界读）'
    )

    #   ⚠️ 用 GroupAction 承载 condition：Humble 的 SetEnvironmentVariable 虽然能吃
    #   condition kwarg，但语义上"整组生效"更清楚，也不会在 gui=True 时白设一遍。
    set_gazebo_offline_env = GroupAction(
        condition=IfCondition(LaunchConfiguration('gazebo_offline')),
        actions=[
            SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', 'http://127.0.0.1:1/'),
        ],
    )

    # Specify the actions
    # ★ 2026-10-07 新增开关（**纯增量**：默认值 = 改造前的行为，不动任何节点的时序/参数）：
    #   · gui=True（默认）      ⇒ 起 gzclient，命令与改造前逐字相同
    #   · gui=False             ⇒ **只起 gzserver**，一条命令无头跑（不用事后 kill GUI）
    #   · gazebo_offline=True   ⇒ 把在线模型库 GAZEBO_MODEL_DATABASE_URI 指到一个必然
    #     立刻失败的地址，避免 gzclient 卡在 "Waiting for model database update to complete..."
    #     （机理与实测见 docs/gazebo_gui_troubleshooting.md）
    #   ⚠️ 为什么不用空串 `GAZEBO_MODEL_DATABASE_URI=""`：gazebo 的
    #     `common::ModelDatabase::GetURI()` 里有 `result[result.size()-1]`，对空串是
    #     越界读（UB）。指向 127.0.0.1:1 是"连接立刻被拒"，语义明确且不碰 UB。
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            # URDF 是 XML，必须显式声明为字符串，否则 launch_ros 会按 YAML 解析而报错
            'robot_description': ParameterValue(robot_description, value_type=str)
        }],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description, value_type=str)
        }],
        output='screen'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', world_config['x'],
                        '-y', world_config['y'],
                        '-z', world_config['z'],
                        '-Y', world_config['yaw'],
                        # RMUL2026 世界加载较慢，默认超时会导致 spawn 提前放弃（实体其实已生成）
                        '-timeout', '60.0',
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': os.path.join(bringup_dir, 'world', world_config['world_path'])}.items(),
                )
            ]
        )

    bringup_RMUC_cmd_group = create_gazebo_launch_group(WorldType.RMUC)
    bringup_RMUL_cmd_group = create_gazebo_launch_group(WorldType.RMUL)
    bringup_RMUL2026_cmd_group = create_gazebo_launch_group(WorldType.RMUL2026)
    bringup_RMUC2026_cmd_group = create_gazebo_launch_group(WorldType.RMUC2026)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(append_enviroment)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(declare_robot_cmd)      # ★ 2026-10-07 模型槽位（默认 '' ⇒ 行为不变）
    ld.add_action(declare_gui_cmd)               # ★ 2026-10-07 gui（默认 True ⇒ 行为不变）
    ld.add_action(declare_gazebo_offline_cmd)    # ★ 2026-10-07 gazebo_offline（默认 False ⇒ 不变）
    # ⚠️ 环境变量必须在 gazebo_client_launch / gzserver 之前生效
    ld.add_action(set_gazebo_offline_env)
    ld.add_action(gazebo_client_launch)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_RMUL_cmd_group) # type: ignore
    ld.add_action(bringup_RMUC_cmd_group) # type: ignore
    ld.add_action(bringup_RMUL2026_cmd_group) # type: ignore
    ld.add_action(bringup_RMUC2026_cmd_group) # type: ignore

    # Uncomment this line if you want to start RViz
    ld.add_action(start_rviz_cmd)

    return ld
