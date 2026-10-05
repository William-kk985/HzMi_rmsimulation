import os
import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, LogInfo, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition
from launch.substitution import Substitution


# =============================================================================
# get_package_share_directory() 的「描述构建期」陷阱（2026-10-05 实测踩到后修复）
# -----------------------------------------------------------------------------
# ❌ 老写法（本文件里三个新槽位原来都这么写）：
#       params = os.path.join(get_package_share_directory('pkg'), 'config', 'x.yaml')
#    这一行在 generate_launch_description() 里 = **描述构建期无条件执行**。
#    于是只要本机没装 'pkg'（没构建过，或构建完**没有重新 source install/setup.bash**），
#    **任何** ros2 launch 组合都会在「解析 launch 文件」这一步直接炸掉 —— 哪怕你选的是
#    `lio:=fastlio localization:=amcl`，跟那个槽位毫无关系：
#        ament_index_python.packages.PackageNotFoundError: "package 'small_point_lio' not found, searching: [...]"
#        launch.invalid_launch_file_error.InvalidLaunchFileError: Caught multiple exceptions ...
#    （`ros2 launch ... --show-args` 同样炸 —— 它也必须先构建 LaunchDescription。）
#    ⚠️ 用户 2026-10-05 就是被这条挡住的：新包还没编进 install/，整个 bringup 一个都起不来。
#
# ✅ 现在：把「查包」推迟到 **真正要用这个路径的那一刻**（= 对应节点被启动时）。
#    `_PackageShareFile` 是一个 launch Substitution，只有 Node.execute() 才会 perform：
#      · 没选该槽位 ⇒ 永不 perform ⇒ 该包不存在也**完全无影响**（--show-args 同理）；
#      · 选了但包/文件不在 ⇒ 抛一条**可操作**的 RuntimeError；launch 会把它打成
#          [ERROR] [launch]: Caught exception in launch (see debug for traceback): <这条消息>
#        （**不是** Python traceback，launch 正常收尾、退出码 1），消息里直接给出
#        `colcon build --symlink-install --packages-select <包名>` 与可替代的槽位值。
#    覆盖的槽位：lio:=small_point_lio、localization:=icp、localization:=gicp | small_gicp
#    （后两者共用 gicp_registration 包）。另外核对了任务点名的两处：
#      · beluga —— 没有任何 get_package_share_directory 调用，只是 IncludeLaunchDescription
#        （在 IfCondition 里 ⇒ 本来就惰性）；参数文件是 rm_navigation 包内的兄弟 YAML。✔ 无需改
#      · small_gicp —— 只是一个字符串 backend 值（库 vendored 在 gicp_registration 内）。✔ 无需改
#    ⚠️ 仍未改的老槽位（fast_lio / point_lio / slam_toolbox / 那批常开节点）**仍是构建期查包**：
#       它们属「基础安装集」，且多处路径要被 os.path.join / PathJoinSubstitution 在构建期拼接，
#       改动面更大 ⇒ 本次不动，登记为已知项（见 docs/lio_slots.md §4 的说明）。
# =============================================================================

# 槽位被选中但包不在时的「替代槽位值」（只进报错消息，不影响任何逻辑）
_SLOT_ALTERNATIVES = {
    'small_point_lio': 'lio:=fastlio（默认值）或 lio:=pointlio',
    'icp_registration': 'localization:=amcl（默认 2D 栅格图路线）或 localization:=gicp',
    'gicp_registration': 'localization:=amcl（默认 2D 栅格图路线）或 localization:=icp',
}


class _PackageShareFile(Substitution):
    """惰性解析「某个包 share 目录下的文件」；缺包/缺文件时给出可操作的报错。

    为什么不用 get_package_share_directory() 直接在构建期拼路径：见上方那段说明。
    它的 perform() 只在「用到这个路径的节点真的被执行」时才会跑 ⇒ 不选这个槽位就零成本。
    """

    def __init__(self, package_name, *relative_path, alternatives=''):
        super().__init__()
        self.__package_name = package_name
        self.__relative_path = relative_path
        self.__alternatives = alternatives or _SLOT_ALTERNATIVES.get(package_name, '')

    def perform(self, context):
        try:
            share_dir = get_package_share_directory(self.__package_name)
        except PackageNotFoundError:
            raise RuntimeError(self.__message(
                '当前 AMENT_PREFIX_PATH 里找不到这个包'
                '（= 没构建过，或构建之后没有重新 source install/setup.bash）'))
        path = os.path.join(share_dir, *self.__relative_path)
        if not os.path.isfile(path):
            raise RuntimeError(self.__message(
                '包在，但里面的文件不在（多半是"加了这个文件之后没重新构建这个包"）：缺失 %s' % path))
        return path

    def __message(self, why):
        alt = ''
        if self.__alternatives:
            alt = '\n  · 或者换一个槽位值：%s' % self.__alternatives
        return (
            "[launch] 缺少本次 launch 需要的 ROS 包 '%(pkg)s'：%(why)s\n"
            '  本次命令**选中了需要这个包的槽位** ⇒ 二选一：\n'
            '  · 构建它并重新 source（推荐）：\n'
            '      cd <你的工作空间> && source /opt/ros/humble/setup.bash\n'
            '      colcon build --symlink-install --packages-select %(pkg)s\n'
            '      source install/setup.bash%(alt)s\n'
            '  （本文件里每个槽位需要的包都是**单独惰性解析**的：缺一个包不会再连带挡住别的组合，'
            '`--show-args` 也不再受影响。）'
            % {'pkg': self.__package_name, 'why': why, 'alt': alt})

    def describe(self):
        return '%s(package=%s, path=%s)' % (
            type(self).__name__, self.__package_name, os.path.join(*self.__relative_path))


def generate_launch_description():
    # Get the launch directory
    rm_nav_bringup_dir = get_package_share_directory('rm_nav_bringup')
    hzmi_rm_simulation_launch_dir = os.path.join(get_package_share_directory('hzmi_rm_simulation'), 'launch')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    spin_speed = LaunchConfiguration('spin_speed')

    ################################ robot_description parameters start ###############################
    # 平台外参（base_link↔livox）随仿真平台包 hzmi_rm_simulation 存放（规则③）
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('hzmi_rm_simulation'), 'config', 'measurement_params_sim.yaml')))
    robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'urdf', 'sentry_robot_sim.xacro'),
    ' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    ################################# robot_description parameters end ################################

    ########################## linefit_ground_segementation parameters start ##########################
    # 参数已回归 linefit_ground_segmentation_ros 包自身 config/（R1）
    segmentation_params = os.path.join(get_package_share_directory('linefit_ground_segmentation_ros'), 'config', 'segmentation_sim.yaml')
    ########################## linefit_ground_segementation parameters end ############################

    #################################### FAST_LIO parameters start ####################################
    # 参数已回归 fast_lio 包自身 config/（R1）
    fastlio_mid360_params = os.path.join(get_package_share_directory('fast_lio'), 'config', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'fastlio.rviz')
    ##################################### FAST_LIO parameters end #####################################

    ################################### POINT_LIO parameters start ####################################
    # 参数已回归 point_lio 包自身 config/（R1）
    pointlio_mid360_params = os.path.join(get_package_share_directory('point_lio'), 'config', 'pointlio_mid360_sim.yaml')
    pointlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'pointlio.rviz')
    #################################### POINT_LIO parameters end #####################################

    ################################# SMALL_POINT_LIO parameters start ###############################
    # 参数放在 vendored 包自身 config/（与 fastlio/pointlio 同一口径：算法参数随包走）
    # ★ 2026-10-05：本槽位改读 **mid360_sim_tuned.yaml**（不是 mid360_sim.yaml）。
    #   原因：mid360_sim.yaml = "仿真输入契约 + 上游默认滤波器参数"，离线重放实测**跟不住**
    #   （40 s 窗口轨迹 30.2 m，同窗口真值 5.51 m / FAST-LIO 参照 6.22 m）；tuned 版只改 4 个
    #   滤波器/点云键（space_downsample=false、imu_meas_omg_cov=0.2、velocity_cov=0.3、
    #   acceleration_cov=50），同一份 bag 上降到 **6.9 m、形状误差 ATE 0.42 m**（FAST-LIO 参照
    #   7.06 m / ATE 0.59 m）。逐键理由、完整 A/B 表与"能不能用"的诚实结论见 docs/lio_slots.md §5.5~§5.8。
    #   回退：把下面这行的文件名换回 'mid360_sim.yaml'（一行改动）。
    # ★ 2026-10-05 修复：路径改为 **惰性解析**（_PackageShareFile）—— 见文件顶部那段说明。
    #   以前这里是无条件 get_package_share_directory('small_point_lio')，机器上没装这个包时
    #   连 `lio:=fastlio localization:=amcl` 都起不来（用户实测 PackageNotFoundError）。
    #   现在只有真选 lio:=small_point_lio 时才会去查包。
    small_point_lio_params = _PackageShareFile(
        'small_point_lio', 'config', 'mid360_sim_tuned.yaml')
    # RViz 直接复用 pointlio 的配置（Fixed Frame = odom，含 TF/Odometry/Path/CloudRegistered 四类显示）
    # —— 本槽位的话题名与点云语义都相同，不值得为它多维护一份 .rviz。
    small_point_lio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'pointlio.rviz')
    ################################## SMALL_POINT_LIO parameters end #################################

    ################################## slam_toolbox parameters start ##################################
    slam_toolbox_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world])
    # slam_toolbox 已源码化，参数回归其自身 config/（R1）
    slam_toolbox_localization_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_localization_sim.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_async_sim.yaml')
    ################################### slam_toolbox parameters end ###################################

    ################################### navigation2 parameters start ##################################
    nav2_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world]), ".yaml"
    empty_map_dir = os.path.join(rm_nav_bringup_dir, 'map', 'empty_map.yaml')
    # nav2 参数已回归自研 rm_navigation 包 params/（R1）。
    # ★ 2026-10-05 params 分层重构（动机/等价性证明见 docs/cod_nav_2026_migration_worksheet.md §J）：
    #   以前 = 「一个组合一份 full copy」（4 nav × 2 planner = 8 份，每份 ~400 行），
    #   代价是改一个公共参数要改 N 份、少一个组合文件就起不来。
    #   现在 = 公共段 + 两个正交槽位，**三份文件按顺序**（base → planner 槽 → controller 槽）传给同一个 nav2 节点：
    #     nav2_params_sim_base.yaml                公共段（除 planner_server / controller_server 外的全部段）
    #     nav2_params_sim_planner_<planner>.yaml   planner_server 段（planner:=navfn | smac2d，默认 navfn）
    #     nav2_params_sim_controller_<nav>.yaml    controller_server 段（nav:=rpp | dwb | teb | mppi，默认 rpp）
    #   ROS 2 的 parameters= 支持多份 YAML **按参数逐键**叠加（同名参数后写覆盖，同一段里没写的键仍来自前一份文件），
    #   每个节点只读自己名字那一段 ⇒ 三段互不冲突。加新规划器/控制器 = 只加一个槽位文件 + 这里一个 choices。
    nav2_params_dir = os.path.join(get_package_share_directory('rm_navigation'), 'params')
    # ⚠️ 这个 list 不能直接塞进 launch_arguments：launch 会把 list 里的 substitution 逐个 perform 后
    #   **拼接成一个字符串**（normalize_to_list_of_substitutions + perform_substitutions）
    #   ⇒ 传下去会变成 "…base.yaml…planner…yaml…controller…yaml" 三串相连的假路径。
    #   所以按槽位拆成三个入参（params_file / params_file_planner / params_file_controller）。
    nav2_params_file_dir = [
        PathJoinSubstitution([nav2_params_dir, 'nav2_params_sim_base.yaml']),
        PathJoinSubstitution([nav2_params_dir, PythonExpression([
            "'nav2_params_sim_planner_' + '", LaunchConfiguration('planner'), "' + '.yaml'"])]),
        PathJoinSubstitution([nav2_params_dir, PythonExpression([
            "'nav2_params_sim_controller_' + '", LaunchConfiguration('nav'), "' + '.yaml'"])]),
    ]
    # 逐槽位传参（老的入参名 params_file 保留 = 公共段 ⇒ 单文件用法仍然可用）
    nav2_params_launch_args = {
        'params_file': nav2_params_file_dir[0],             # 公共段（base）
        'params_file_planner': nav2_params_file_dir[1],     # planner_server 槽
        'params_file_controller': nav2_params_file_dir[2],  # controller_server 槽
    }
    # ★ 2026-10-05：localization:=beluga 槽（beluga_amcl，AMCL 的现代实现，Apache-2.0）。
    #   位置选择：**兄弟参数文件**而不是复用 base 的 amcl 段 —— 两者参数集不是超集关系
    #   （nav2 有 do_beamskip/beam_skip_*/save_pose_rate/initial_pose.z；beluga 有
    #    initial_pose.covariance_*/spatial_resolution_*/model_unknown_space 等），
    #   ROS 2 对未声明的键是**静默忽略** ⇒ 复用会同时留下无效键与缺失键。
    #   逐键映射表见 docs/localization_slots.md §1.2。
    beluga_params_file_dir = os.path.join(nav2_params_dir, 'nav2_params_sim_beluga.yaml')
    # ⚠️ 旧的 nav2_params_sim_<nav>[_smac2d].yaml（8 份 full copy）**已废弃**：本 launch 不再引用它们，
    #   文件保留只为「参考 / 回滚」（等价性已逐键核实：8/8 组合 0 处差异，见 docs §J）。
    # AMCL 初值（map 系，米/弧度）：sim 出生点固定，按 world 自动注入，省掉手动发 /initialpose。
    # 依据（用场地 STL 世界包围盒 vs pgm 已知区域比对得出，见 docs/smoke_test_runbook.md §0.5）：
    #   RMUC / RMUL 的 pgm 是 sim 建图导出 → map 原点 = 机器人出生点 → (0, 0, 0)
    #   RMUL2026 的 pgm 是场地几何生成     → map 系 = world 系     → (4.3, 3.35, 0)
    # ★ 2026-09-23：RMUL2026 的 pgm 已换成 **cartographer 新建的图**（见 docs/mapping/README.md
    #   "本次落盘记录"）。cartographer 的 map 系 = **出生点相对系**（新 yaml origin ≈[-2.2,-3.15]，
    #   与旧世界系图差 ≈(4.3,3.35)）⇒ 新图下出生点在 map 里的坐标就是 (0,0,0)。
    #   ⚠️ 若把旧图放回（map/RMUL2026_world_backup.*），这两个值要改回 4.3 / 3.35。
    # ★ 2026-10-05 新增 world:=RMUC2026：它的 map/PCD 是**按出生点相对系**生成的
    #   （pgm origin = [场地左下角 - 出生点] = [-25.925, -9.425]，出生点在 map 里就是原点）
    #   ⇒ 与 RMUL2026 同一口径：0.0 / 0.0。依据见 docs/worlds.md §3~§4。
    amcl_init_x = PythonExpression([
        "{'RMUC': 0.0, 'RMUL': 0.0, 'RMUL2026': 0.0, 'RMUC2026': 0.0}['",
        LaunchConfiguration('world'), "']"])
    amcl_init_y = PythonExpression([
        "{'RMUC': 0.0, 'RMUL': 0.0, 'RMUL2026': 0.0, 'RMUC2026': 0.0}['",
        LaunchConfiguration('world'), "']"])
    ################################### navigation2 parameters end ####################################

    # cartographer 纯定位的资产：map/<world>.pbstream（由 cartsographer 建图 + write_state 生成）
    carto_pbstream_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world]), ".pbstream"

    ################################ icp_registration parameters start ################################
    icp_pcd_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'PCD', world]), ".pcd"
    # FAST-LIO 的 pcd 落盘路径（/map_save 服务写入），与 ICP 的底图同一路径 → 建图产物直接可被复用
    fastlio_pcd_out = PathJoinSubstitution([
        rm_nav_bringup_dir, 'PCD',
        PythonExpression(["'", LaunchConfiguration('world'), "' + '.pcd'"])])
    # 参数已回归 icp_registration 包自身 config/（R1），经 ament_auto_package INSTALL_TO_SHARE 安装
    # ★ 2026-10-05：与 small_point_lio 同一类修复 —— 惰性解析，缺包不再挡住别的组合（见文件顶部）
    icp_registration_params_dir = _PackageShareFile(
        'icp_registration', 'config', 'icp_registration_sim.yaml')
    # ★ 2026-10-05：新增 localization:=gicp 槽（GICP 精配准）—— 与 icp 槽同一份资产（PCD/<world>.pcd），
    #   参数回归 gicp_registration 包自身 config/（见 docs/localization_slots.md §1）
    # ★ 2026-10-05 同日：新增 localization:=small_gicp 槽 —— **与 gicp 槽共用这一份参数文件**，
    #   唯一差别是 launch 注入 backend='small_gicp'（文件里的默认值仍是 "pcl"）。
    #   ⇒ 两个槽位各自一行、互不干扰；文件里那个 backend 默认值**不代表推荐哪一个**，
    #     而是"单独跑本节点 / 不经验收的用法"时的保守默认（依据见
    #     docs/gicp_backend_small_gicp.md：两个后端是**对等**取舍）。
    # ★ 2026-10-05：同一类修复 —— 惰性解析，缺包只挡住 localization:=gicp / small_gicp 这两个槽位
    gicp_registration_params_dir = _PackageShareFile(
        'gicp_registration', 'config', 'gicp_registration_sim.yaml')
    ################################# icp_registration parameters end #################################

    # Declare launch options
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO or Point_LIO cloud_map if true')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    declare_spin_speed_cmd = DeclareLaunchArgument(
        'spin_speed',
        default_value='5.0',
        description='fake_vel_transform 的小陀螺固定角速度 (rad/s)。'
                    '5.0 = 复现上游哨兵小陀螺行为；0.0 = 角速度直通（等价普通 nav2，'
                    '仿真里排查导航问题先用 0.0）')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='RMUL2026',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='场景形态（必填，三选一）: '
                    'mapping = 纯建图（Gazebo + LIO + 在线 SLAM 后端；【不起导航栈】）| '
                    'slam_nav = 边建图边导航（在线 SLAM 直接把 /map 与 map→odom 喂给 costmap；'
                    '不加载磁盘地图、不起任何重定位模块）| '
                    'nav = 先建图后导航（加载磁盘地图 + 必须指定 localization 重定位模块）')

    # ★ 2026-10-05：新增槽位值 'small_gicp'。三个 ICP 族重定位器（icp / gicp / small_gicp）在
    #   **槽位层面是三个并列的值**：跑 bench 的人只选一个，不必知道也不必配置"它是怎么实现的"
    #   （gicp / small_gicp 共用 gicp_registration 这个节点、共用同一套 GICP 数学、只换库 ——
    #    其中的 backend 由下面各自的 launch 分支**注入**，无需改 YAML、无需 -p backend:=...）。
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        choices=['', 'amcl', 'beluga', 'slam_toolbox', 'icp', 'gicp', 'small_gicp', 'cartographer'],
        description='仅 mode:=nav 生效。重定位模块（选一个即可，实现细节无需关心）: '
                    'amcl（2D 栅格图）| beluga（beluga_amcl —— AMCL 的现代实现，'
                    '与 amcl 同资产（2D 栅格图）/同契约/同 lifecycle 形态，需 ros-humble-beluga-amcl）| '
                    'slam_toolbox（需 map/<world>.posegraph）| '
                    'icp（PCL ICP，需 PCD/<world>.pcd，初值敏感）| '
                    'gicp（PCL GICP，需 PCD/<world>.pcd，初值敏感）| '
                    'small_gicp（small_gicp 实现，需 PCD/<world>.pcd，初值敏感）| '
                    'cartographer（纯定位，需 map/<world>.pbstream）；'
                    'icp / gicp / small_gicp 三者都吃同一份 PCD/<world>.pcd，'
                    '初值都给 /initialpose（RViz 2D Pose Estimate）或 initial_pose 参数；'
                    '留空 = 回退用法，直接用 LIO 当绝对定位并由静态桥补帧')

    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fastlio',
        # ★ 2026-10-05：新增 'small_point_lio'（RM26 东莞理工 ACE 的 Point-LIO 2~3× 加速变体，
        #   vendored MIT，pin 688d75c；与 fastlio/pointlio 同级、可 A/B。默认值不变。）
        choices=['fastlio', 'pointlio', 'small_point_lio', 'none', 'cartographer'],
        description='里程计源（谁发 odom→base_link）: fastlio | pointlio | '
                    'small_point_lio（vendored Yancey2023/small_point_lio，**自己直发 odom→base_link，'
                    '因此不经过 lio_tf_adapter**；无 /odom 之外的 path 话题）| '
                    'none（不启动 LIO，需外部提供 odom/TF，如轮式里程计）| '
                    'cartographer（**全包形态**：cartographer 兼任里程计源 → 同时跳过 mapper 槽与 '
                    'localization 槽；mode:=nav 时用 map/<world>.pbstream 做纯定位。'
                    '它不发 /odom 话题 → nav:=teb 不适用，用 rpp/dwb。见 docs/tf_interface_contract.md；'
                    'small_point_lio 的契约、验证与回退见 docs/lio_slots.md）')

    declare_nav_cmd = DeclareLaunchArgument(
        'nav',
        default_value='rpp',
        # ★ 2026-09-26：新增 'mppi'（A/B 对照用；默认仍是 rpp ⇒ 现有已验证路径零影响）。
        # ★ 2026-10-05：nav 现在只决定 **controller 槽位文件**（nav2_params_sim_controller_<nav>.yaml），
        #   由它与公共段 base 组合，不再是「一个组合一份 full copy」（见 docs §J）。
        choices=['rpp', 'dwb', 'teb', 'mppi'],
        description='Choose local planner variant: rpp | dwb | teb | mppi '
                    '(对应 rm_navigation/params/nav2_params_sim_controller_<nav>.yaml，'
                    '与公共段 nav2_params_sim_base.yaml 按顺序叠加；'
                    'mppi = nav2_mppi_controller::MPPIController，A/B 用；回退 = 省略本参数或 nav:=rpp)')

    declare_planner_cmd = DeclareLaunchArgument(
        'planner',
        default_value='navfn',
        choices=['navfn', 'smac2d'],
        description='全局规划器槽位（A/B 对照用，只换 planner_server 的 GridBased，其它一律不动）: '
                    'navfn = 已验证默认（不要动；nav2_navfn_planner/NavfnPlanner）；'
                    'smac2d = COD 2026 风格的代价感知 A*（nav2_smac_planner/SmacPlanner2D，'
                    'cost_travel_multiplier 越大越贴通道中心），只用于 A/B。'
                    '对应 rm_navigation/params/nav2_params_sim_planner_navfn.yaml 与 '
                    'nav2_params_sim_planner_smac2d.yaml（planner_server 槽位，与公共段 base 叠加）；'
                    '回退 = 省略本参数或 planner:=navfn')

    declare_global_obstacle_cmd = DeclareLaunchArgument(
        'global_obstacle',
        default_value='stvl',
        description='全局代价地图实时障碍来源（bench 可切换槽位）: '
                    'stvl = 3D 体素层(STVL，吃 /segmentation/obstacle，默认保持原行为) | '
                    'scan = 2D 障碍层(吃 /scan，与 local_costmap 同源，把"什么算障碍"收敛到感知域 p2l 一处) | '
                    'none = 只用 static+inflation（全局看不到实时障碍）')

    declare_local_obstacle_cmd = DeclareLaunchArgument(
        'local_obstacle',
        default_value='scan',
        choices=['scan', 'cloud', 'both', 'stvl', 'stvl_both'],
        description='局部代价地图障碍来源（bench 可切换槽位）: '
                    'scan = 只吃 /scan（默认，原行为；链路单点） | '
                    'cloud = 只吃 /segmentation/obstacle 点云直投（不经 p2l，无近距盲区） | '
                    'both = 双源冗余（scan + cloud，任一路挂掉仍能避障） | '
                    'stvl = 3D 体素层（SpatioTemporalVoxelLayer，吃 /segmentation/obstacle，'
                    '带 0.5s 时间衰减，与 global_obstacle:=stvl 同款 = COD 2026 的双图做法） | '
                    'stvl_both = stvl + scan + cloud（冗余最多、CPU 最贵；未做 bench 实测）。'
                    '开关表 = rm_navigation/launch/navigation_launch.py 的 LOCAL_OBSTACLE_LAYER_TABLE；'
                    '实测见 docs/stvl_local_costmap.md')

    declare_mapper_cmd = DeclareLaunchArgument(
        'mapper',
        default_value='slam_toolbox',
        choices=['cartographer', 'slam_toolbox'],
        description='Choose 2D mapping backend (only mode:=mapping): slam_toolbox | cartographer')

    # Specify the actions
    start_rm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(hzmi_rm_simulation_launch_dir, 'rm_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_description': robot_description,
            # ★ 2026-09-24：nav2 不共用组合容器（use_composition=False）。
            #   当时的假设：两张 costmap 同处一个 component_container_mt 会互相拖死（摄入静默冻死）。
            #   ⚠️ **该假设已被后续实测否定**：拆成独立进程后同样复现；真因是 livox 插件的 sensor 回调被
            #   CustomMsg 的 RELIABLE 背压锁死 ⇒ /livox/lidar 与 /livox/lidar/pointcloud 同时停发 ⇒ 没有 /odom
            #   ⇒ TF 里不存在 odom 帧 ⇒ costmap 的 getRobotPose() 取到的「最新公共时刻」被钉死在启动那一刻
            #   （published_footprint 恒为 644.682，而它自己的 costmap_raw 戳是新鲜的）⇒ planner/controller 拿过期 TF
            #   ⇒ 撞上上游 nav2 `isGoalReached()` 丢弃 transformPose 返回值那个 bug ⇒ 目标被当成 (0,0,0) ⇒ 假"到达"、车不动。
            #   真因与修法见 docs/debug_fastlio_cartographer.md §9.2 候选④（CustomMsg/订阅 QoS 已于 2026-09-24 修复）。
            #   本行保留 False 属「故障隔离」的保守选择，代价是多几个进程；想省进程可删掉本行，但需重跑验收。
            'use_composition': 'False',
            'rviz': 'False'}.items()
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        # 参数已回归 imu_complementary_filter 包 config/（R1）
        parameters=[os.path.join(get_package_share_directory('imu_complementary_filter'), 'config', 'imu_filter_params.yaml')],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
    )

    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        # ★ 2026-09-23 修复：原来漏了 use_sim_time ⇒ 这个节点跑在**墙钟**上，而全链路（plugin/scan/
        #   costmap/AMCL/tf）都是仿真钟。它的输出戳虽然抄自输入（所以看起来还好），但任何依赖
        #   "本节点时钟"的逻辑（tf2 Buffer 的缓存窗口、超时判定）都会用错时间轴。
        parameters=[segmentation_params, {'use_sim_time': use_sim_time}]
    )

    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
        # 参数已回归 pointcloud_to_laserscan 包 config/（R1）
        # ★ 2026-09-23 修复：同上，原来漏了 use_sim_time。p2l 内部用
        #   `tf2_ros::Buffer(this->get_clock())` 建 TF 缓存，节点时钟是墙钟时缓存窗口走的是墙钟，
        #   与消息里的仿真戳不在一条时间轴上（本配置 target_frame==点云 frame，过滤器短路才没炸；
        #   一旦改 target_frame 就会立刻表现为"整条 Talker 被丢光"）。
        parameters=[os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'config', 'laserscan_params.yaml'),
                    {'use_sim_time': use_sim_time}],
        name='pointcloud_to_laserscan'
    )

    bringup_LIO_group = GroupAction([
        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'fastlio'),
            actions=[
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[
                    fastlio_mid360_params,
                    # ★ 2026-09-24 修复：原来键名漏了引号（{use_sim_time: use_sim_time}）⇒ 这个参数
                    #   根本没传到节点（实测 /laser_mapping use_sim_time=False，而全栈其它节点都是 True）。
                    {'use_sim_time': use_sim_time},
                    {'runtime_pos_log_enable': False},
                    # 装配级：pcd 落盘路径按 world 自动指向 PCD/<world>.pcd
                    # （= icp_registration 的底图路径，建图产物直接可被重定位复用）。
                    # FAST-LIO 只在 /map_save 服务被调用时写这个文件，启动不加载它，所以不会"接着旧图建"。
                    {'map_file_path': fastlio_pcd_out}
                ],
                output='screen',
                # T2：统一里程计话题为 /odom（源话题 /Odometry）
                remappings=[('/Odometry', '/odom')],
                # 添加额外的参数以避免崩溃
                arguments=['--ros-args', '--log-level', 'info']
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', fastlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            ),
        ]),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'pointlio'),
            actions=[
            Node(
                package='point_lio',
                executable='pointlio_mapping',
                name='laserMapping',
                output='screen',
                parameters=[
                    pointlio_mid360_params,
                    # 算法调参键已并入 pointlio_mid360_sim.yaml（R1），此处仅留装配级 use_sim_time
                    {'use_sim_time': use_sim_time}
                ],
                # T2：统一里程计话题为 /odom（源话题 aft_mapped_to_init）
                remappings=[('/aft_mapped_to_init', '/odom'), ('aft_mapped_to_init', '/odom')],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', pointlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            )
        ]),

        # ===== lio:=small_point_lio（vendored Yancey2023/small_point_lio，MIT，pin 688d75c）=====
        # 与上面两个槽位的**本质区别**：它自己就发 `odom→base_link`（源码硬编码，
        # small_point_lio_node.cpp:61-62），所以：
        #   ① /Odometry → /odom 仍要 remap（统一话题口径）；
        #   ② **绝不能**再起 lio_tf_adapter（否则 odom→base_link 两个发布者 → 双父边/抖动）；
        #   ③ 它不认 camera_init/body ⇒ T1 回退静态桥也要排除它（见下面 icp_frame_bridge_condition）。
        # 它**需要** base_link→livox_frame 这条静态 TF（源码 lookupTransform(lidar_frame,"base_link")，
        # 失败就丢帧、连 /Odometry 都不发）—— 仿真里由 robot_state_publisher 从 URDF 提供
        # （实测 /tf_static 里有 base_link→livox_frame (0.12,0,0.175)，见 docs/lio_slots.md §2）。
        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'small_point_lio'),
            actions=[
            # 把**实际加载的参数文件**打进日志（惰性解析 ⇒ 正好在这里才求值）：
            # 用户不必去猜"launch 到底给我读了 mid360_sim.yaml 还是 tuned 版"，日志里直接有。
            LogInfo(msg=['lio:=small_point_lio 使用的参数文件 = ', small_point_lio_params]),
            Node(
                package='small_point_lio',
                executable='small_point_lio_node',
                name='small_point_lio',
                output='screen',
                parameters=[
                    small_point_lio_params,
                    # 装配级：use_sim_time（算法参数都在 mid360_sim.yaml 里）
                    {'use_sim_time': use_sim_time}
                ],
                # T2：统一里程计话题为 /odom（源话题硬编码为 /Odometry，见 small_point_lio_node.cpp:26）
                remappings=[('/Odometry', '/odom')],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', small_point_lio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            ),
        ])
    ])

    # ===== lio:=cartographer（全包形态）=====
    # cartographer 同时发 odom→base_link 与 map→odom（见 configuration_files/cartographer_lio*.lua），
    # 因此：① mapper 槽、localization 槽都要跳过（否则地图/TF 出现第二个发布者）；
    #       ② lio_tf_adapter 与 T1 回退静态桥（body→odom）也必须关，否则 odom 多父边。
    carto_as_lio = ["'", LaunchConfiguration('lio'), "' == 'cartographer'"]
    carto_as_lio_mapping_condition = IfCondition(PythonExpression(
        carto_as_lio + [" and '", LaunchConfiguration('mode'), "' != 'nav'"]))
    carto_as_lio_nav_condition = IfCondition(PythonExpression(
        carto_as_lio + [" and '", LaunchConfiguration('mode'), "' == 'nav'"]))

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition = IfCondition(PythonExpression([
                    "'", LaunchConfiguration('localization'), "' == 'slam_toolbox' and '",
                    LaunchConfiguration('lio'), "' != 'cartographer'"])),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [0.0, 0.0, 0.0]}
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,'localization_amcl_launch.py')),
                condition = IfCondition(PythonExpression([
                    "'", LaunchConfiguration('localization'), "' == 'amcl' and '",
                    LaunchConfiguration('lio'), "' != 'cartographer'"])),
                # amcl_launch 现在同时启动 map_server + amcl（同一个 lifecycle_manager），故需传入地图
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    **nav2_params_launch_args,
                    'initial_pose_x': amcl_init_x,
                    'initial_pose_y': amcl_init_y,
                    'initial_pose_z': '0.0',
                    'initial_pose_yaw': '0.0'}.items()
            ),

            # ★ 2026-10-05：localization:=beluga —— beluga_amcl（Ekumen-OS/beluga，Apache-2.0）。
            #   与 amcl 槽**同形态**：本 include 自己起 map_server + 定位节点 + lifecycle_manager
            #   （node_names = ['map_server','amcl']）⇒ 「map_server 照常在跑」这条契约与 amcl 一致，
            #   因此下面那条独立 map_server include 必须把 beluga 也排除掉（否则 /map 两个发布者）。
            #   只发 map→odom；/scan 是 BEST_EFFORT（beluga 内部用 SensorDataQoS 订阅）。
            #   参数文件 = 兄弟文件 nav2_params_sim_beluga.yaml（映射表见 docs/localization_slots.md §1.2）。
            #   ⚠️ 不给 initial_pose_z：beluga 的 2D 节点没有这个键（nav2_amcl 才有）。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'localization_beluga_launch.py')),
                condition = IfCondition(PythonExpression([
                    "'", LaunchConfiguration('localization'), "' == 'beluga' and '",
                    LaunchConfiguration('lio'), "' != 'cartographer'"])),
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    **nav2_params_launch_args,
                    'beluga_params_file': beluga_params_file_dir,
                    'initial_pose_x': amcl_init_x,
                    'initial_pose_y': amcl_init_y,
                    'initial_pose_yaw': '0.0'}.items()
            ),

            # localization:=cartographer —— 纯定位（加载 .pbstream，frozen state）
            # 与本工程契约一致：只发 map→odom（lua 里 published_frame="odom" + provide_odom_frame=false），
            # odom→base_link 仍由 LIO 提供。栅格发到 /cartographer_map，把 /map 让给 map_server 的先验图，
            # 避免 /map 双发布者（map_server 的启动条件见下面那段 Include）。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(rm_nav_bringup_dir, 'launch', 'cartographer_sim.launch.py')),
                condition = IfCondition(PythonExpression([
                    "'", LaunchConfiguration('localization'), "' == 'cartographer' and '",
                    LaunchConfiguration('lio'), "' != 'cartographer'"])),
                launch_arguments = {
                    'configuration_basename': 'cartographer_localization.lua',
                    'load_state_filename': carto_pbstream_dir,
                    'load_frozen_state': 'true',
                    'occupancy_grid_topic': '/cartographer_map'}.items()
            ),

            # lio:=cartographer（全包形态）+ mode:=nav —— cartographer 自己做纯定位**并且**兼任里程计源。
            # 与上面那条的区别只有两点：lua 用 cartographer_lio_localization.lua（provide_odom_frame=true），
            # 且由它自己发 odom→base_link（所以 lio 不能再是 fastlio/pointlio）。
            # /map 仍由 map_server 发先验图（map_server 的条件只看 localization != amcl/slam_toolbox，
            # 而本形态要求 localization 留空 → 条件成立，符合预期）。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(rm_nav_bringup_dir, 'launch', 'cartographer_sim.launch.py')),
                condition = carto_as_lio_nav_condition,
                launch_arguments = {
                    'configuration_basename': 'cartographer_lio_localization.lua',
                    'load_state_filename': carto_pbstream_dir,
                    'load_frozen_state': 'true',
                    'occupancy_grid_topic': '/cartographer_map',
                    # 全包形态没有 LIO 的 /odom → 用仿真底盘里程计喂 use_odometry（实车换成下位机轮速）
                    'odom_topic': '/odom_ground_truth'}.items()
            ),

            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        condition=IfCondition(PythonExpression([
                            "'", LaunchConfiguration('localization'), "' == 'icp' and '",
                            LaunchConfiguration('lio'), "' != 'cartographer'"])),
                        package='icp_registration',
                        executable='icp_registration_node',
                        output='screen',
                        parameters=[
                            icp_registration_params_dir,
                            {'use_sim_time': use_sim_time,
                                'pcd_path': icp_pcd_dir}
                        ],
                        # arguments=['--ros-args', '--log-level', ['icp_registration:=', 'DEBUG']]
                    ),

                    # localization:=gicp —— GICP 精配准（与 icp 槽同资产、同初值契约、同 IfCondition 形态）。
                    # 契约：只发 map→odom；**必须有初值**（/initialpose 或 config 里的 initial_pose），
                    # 否则节点不发 TF 并在日志里每 2 s 说明一次（不会静默发垃圾 TF）。
                    # 健康信号：/gicp_registration/{fitness_score,converged,pose}（见 docs/localization_slots.md §1）。
                    Node(
                        condition=IfCondition(PythonExpression([
                            "'", LaunchConfiguration('localization'), "' == 'gicp' and '",
                            LaunchConfiguration('lio'), "' != 'cartographer'"])),
                        package='gicp_registration',
                        executable='gicp_registration_node',
                        output='screen',
                        parameters=[
                            gicp_registration_params_dir,
                            {'use_sim_time': use_sim_time,
                                'pcd_path': icp_pcd_dir}
                        ],
                        # arguments=['--ros-args', '--log-level', ['gicp_registration:=', 'DEBUG']]
                    ),

                    # localization:=small_gicp —— **与 gicp 槽并列的第三个 ICP 族槽位**：
                    # 同一个包、同一个节点、同一套 GICP 数学、同一份资产与初值契约，
                    # 唯一区别是配准后端换成 vendored koide3/small_gicp（MIT，OpenMP 多线程）。
                    # ★ 槽位用户**不需要**知道这一点：backend 由本分支注入（就在下面那个 dict 里），
                    #   既不用改 config/gicp_registration_sim.yaml（那里的默认仍是 "pcl"），
                    #   也不用 -p backend:=small_gicp。选 localization:=small_gicp 就够了。
                    # ★ 故意不做成"一个槽 + 一个 backend 参数"：体验者要的是"选一个重定位器"，
                    #   两个实现必须在槽位层面分得清清楚楚（A/B 也各自可复现）。
                    # 契约与 gicp 槽逐条相同：只发 map→odom；必须有初值（/initialpose 或
                    #   initial_pose 参数），否则不发 TF 并每 2 s 说明一次。
                    # 健康信号：/gicp_registration/{fitness_score,converged,pose}
                    #   （backend=small_gicp 时另加 ~/small_gicp_error，非 m²，别混用）。
                    Node(
                        condition=IfCondition(PythonExpression([
                            "'", LaunchConfiguration('localization'), "' == 'small_gicp' and '",
                            LaunchConfiguration('lio'), "' != 'cartographer'"])),
                        package='gicp_registration',
                        executable='gicp_registration_node',
                        output='screen',
                        parameters=[
                            gicp_registration_params_dir,
                            {'use_sim_time': use_sim_time,
                                'pcd_path': icp_pcd_dir,
                                # ★ 本槽位的全部"实现细节"就在这一行：由 launch 注入，无需人工配置
                                'backend': 'small_gicp'}
                        ],
                    ),
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                # 仅 nav 模式 + icp/gicp/small_gicp（或未选重定位）时才单独起 map_server。
                # amcl / beluga 槽**自带** map_server（与定位节点同一个 lifecycle_manager），
                # slam_toolbox 自己发 /map ⇒ 这三种都不能再起第二个 map_server（/map 双发布者 +
                # 同名 lifecycle_manager_localization 冲突 ⇒ map_server 无法激活）。
                # ⚠️ 三个 ICP 族槽位（icp / gicp / small_gicp）**都**需要这个独立 map_server
                #   （它们只发 map→odom、不发 /map），因此下面这条排除条件**只排除 amcl/beluga/slam_toolbox**。
                # ⚠️ 必须带 mode=='nav'：建图模式下 localization 为空，若不判断 mode，
                # map_server 会把【磁盘上的旧 pgm】发到 /map，与 slam_toolbox/cartographer
                # 抢同一个话题，导致 map_saver_cli 可能存下旧图（幽灵墙就是这么留下的）。
                condition = IfCondition(PythonExpression([
                    "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                    LaunchConfiguration('localization'), "' != 'slam_toolbox' and '",
                    LaunchConfiguration('localization'), "' != 'amcl' and '",
                    LaunchConfiguration('localization'), "' != 'beluga'"])),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    **nav2_params_launch_args,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    bringup_fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        # spin_speed 已回归 fake_vel_transform 包 config/（R1），此处仅留装配级 use_sim_time。
        # spin_speed 是「小陀螺」语义：nav2 只要发了非零角速度，底盘就按该固定角速度旋转
        # （真实机器人上电控本来就在自转，此值用于增减；仿真里没有云台补偿，雷达会跟着底盘一起转）。
        # 仿真调参建议：先用 spin_speed:=0.0（角速度直通，等价普通 nav2）把导航链路跑通，
        # 再打开 5.0 复现小陀螺行为做对比实验。
        parameters=[os.path.join(get_package_share_directory('fake_vel_transform'), 'config', 'fake_vel_params.yaml'),
                    {'use_sim_time': use_sim_time,
                     'spin_speed': spin_speed}]
    )

    # T3：LIO 位姿 → 标准帧树适配（odom→base_link），启用 LIO 时启动
    # （配合 T4 关闭 Gazebo 的 odom→base_link，使其成为唯一位姿来源；T2 已把里程计话题统一为 /odom）
    # ★ 2026-10-05：新增排除 'small_point_lio' —— 它**自己就发 odom→base_link**（源码硬编码），
    #   再叠加本适配器就是两个发布者（= 我们文档里反复强调的多父边）。
    #   它不需要适配器的另一个理由：适配器存在的意义是（a）把 LIO 的话题改名成 /odom、
    #   （b）补掉 body/imu_link 与 base_link 之间 0.125 m 杆臂；而 small_point_lio 是
    #   **先在节点内部用 TF 把雷达位姿换算到 base_link** 再发 TF 的（small_point_lio_node.cpp:63-76），
    #   杆臂由它自己处理（⚠️ 该换算的写法有瑕疵，见 docs/lio_slots.md §6 第 2 条）。
    lio_tf_adapter_node = Node(
        condition = IfCondition(PythonExpression([
            "'", LaunchConfiguration('lio'), "' != 'none' and '",
            LaunchConfiguration('lio'), "' != 'cartographer' and '",
            LaunchConfiguration('lio'), "' != 'small_point_lio'"])),
        package='lio_tf_adapter',
        executable='lio_tf_adapter_node',
        name='lio_tf_adapter',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('lio_tf_adapter'), 'config', 'lio_tf_adapter.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # 注（2026-09-22 更正，原文写反了）：`body` **并不在 URDF 里**（曾试图用 imu_link→body 固定关节
    # 兜住 LIO 的 odom child_frame_id，但会造成 body 双父边，已撤销 —— 见 issues_and_findings.md #19/#21）。
    # 现在的做法：LIO 的 odom `child_frame_id` 直接写 `imu_link`（FAST_LIO laserMapping.cpp:631）→
    # **不需要任何 body 桥**。FAST-LIO 仍会发一条 `camera_init→body`，那是**孤立岛（无父）**，无害。
    # 真正要防的是"多父边"：`map`/`odom` 一旦出现两个父，tf2 的查找结果会在两条路径间跳
    # → 症状是 map→odom 高频甩动 + 几十度大跳（与"SLAM 在矫正"很容易混淆）。

    # T1（修正版）：帧桥只在「nav + 未选择任何重定位模块 + 启用 LIO」时启动，
    # 即把 LIO 当作绝对定位（map≡camera_init、odom≡body）的回退用法。
    # amcl / beluga(= beluga_amcl) / slam_toolbox / icp_registration / gicp_registration
    # （= 槽位 gicp 与 small_gicp —— 两者是同一个节点、只换 backend）都会自行
    # 发布 map→odom，绝不能再叠加静态桥（否则 map/odom 多父边）。
    # ★ 2026-10-05：再排除 'small_point_lio' —— 它既不发 camera_init→body（那条链的两个帧都不存在），
    #   又把 odom 直接当世界系（自己发 odom→base_link）⇒ 它需要的是 **map≡odom 单条静态桥**，
    #   见下面的 tf_bridge_spl_map_to_odom_node。
    icp_frame_bridge_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'nav' and '",
        LaunchConfiguration('localization'), "' == '' and '",
        LaunchConfiguration('lio'), "' != 'none' and '",
        LaunchConfiguration('lio'), "' != 'cartographer' and '",
        LaunchConfiguration('lio'), "' != 'small_point_lio'"]))

    tf_bridge_node = Node(
        condition=icp_frame_bridge_condition,
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_bridge_camera_init_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    tf_bridge_node2 = Node(
        condition=icp_frame_bridge_condition,
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_bridge_body_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # lio:=small_point_lio + localization:=''（回退用法：LIO 当绝对定位）的帧桥。
    # 它的 world 帧就叫 odom（自己发 odom→base_link，没有 camera_init/body）⇒ map≡odom 恒等即可。
    # 只发 map→odom 一条：绝不发 camera_init/body 桥（那两个帧在本槽位根本不存在，发了就是孤立岛）。
    tf_bridge_spl_map_to_odom_node = Node(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'nav' and '",
            LaunchConfiguration('localization'), "' == '' and '",
            LaunchConfiguration('lio'), "' == 'small_point_lio'"])),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_bridge_spl_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 注：base_link→base_link_fake 由 fake_vel_transform 以 20Hz 发布（含云台转角），
    # 原先这里的静态桥是重复发布（P4），已删除。
    
    # 在mapping模式下也需要启动map_server来显示静态地图（如果存在）
    # 但只在nav模式下才加载静态地图进行定位
    # 对于mapping模式，我们主要依赖LIO的点云地图
    # ===== 场景形态（mode）三种，启动集明显不同 =====
    #   mapping  : Gazebo + LIO(+RViz) + 在线 SLAM 后端         —— 无导航栈、无地图加载、无重定位
    #   slam_nav : 上述 + 导航栈（costmap 直接吃在线 SLAM 的 /map 与 map→odom）—— 无 map_server、无重定位
    #   nav      : Gazebo + LIO + 重定位(amcl/beluga/slam_toolbox-loc/icp/gicp/small_gicp/cartographer) + 导航栈 —— 无在线 SLAM 后端
    mode_nav = ["'", LaunchConfiguration('mode'), "' == 'nav'"]
    mode_slam_nav = ["'", LaunchConfiguration('mode'), "' == 'slam_nav'"]
    mode_mapping = ["'", LaunchConfiguration('mode'), "' == 'mapping'"]

    # 导航栈：先建后导(nav) + 边建边导(slam_nav)；纯建图(mapping)不启动，省下 7 个 nav2 节点与两张 costmap
    nav_stack_condition = IfCondition(PythonExpression(
        ['('] + mode_nav + [' or '] + mode_slam_nav + [')']))

    # 在线 SLAM 后端（slam_toolbox async / cartographer 在线建图）：纯建图 + 边建边导
    online_mapping_condition = IfCondition(PythonExpression(
        ['('] + mode_mapping + [' or '] + mode_slam_nav + [')']))

    # 2D 建图后端二选一（mapper:=slam_toolbox|cartographer，mode:=mapping 或 slam_nav 生效）
    # 注：lio:=cartographer（全包形态）时 mapper 槽整体跳过 —— 那一个 cartographer 已经在建图并发 /map。
    slam_mapping_condition = IfCondition(PythonExpression(
        ['('] + mode_mapping + [' or '] + mode_slam_nav + [") and '",
         LaunchConfiguration('mapper'), "' == 'slam_toolbox' and '",
         LaunchConfiguration('lio'), "' != 'cartographer'"]))
    carto_mapping_condition = IfCondition(PythonExpression(
        ['('] + mode_mapping + [' or '] + mode_slam_nav + [") and '",
         LaunchConfiguration('mapper'), "' == 'cartographer' and '",
         LaunchConfiguration('lio'), "' != 'cartographer'"]))

    start_mapping = Node(
        condition = slam_mapping_condition,
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
    )

    start_cartographer_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rm_nav_bringup_dir, 'launch', 'cartographer_sim.launch.py')),
        condition = carto_mapping_condition,
        launch_arguments={
            'configuration_basename': 'cartographer.lua',
            # ★ 先验来源必须是**独立的**底盘/轮速里程计，不能用 LIO 自己的 /odom：
            #   同源（同一份点云）+ 晚到（LIO 处理完整帧才发）→ 撞 cartographer 的时间序 CHECK
            #   → exit -6(SIGABRT)，并形成反馈回路。详见 cartographer.lua 顶部注释、
            #   docs/issues_and_findings.md #21、docs/sim_real_contract.md §七.5。
            #   仿真 = /odom_ground_truth（planar_move）；实车 = 下位机轮速 odom。
            'odom_topic': '/odom_ground_truth'}.items()
    )

    # lio:=cartographer（全包形态）+ mapping/slam_nav —— 同一个 cartographer 既建图又当里程计源。
    # mapper 槽此时被跳过（上面两个 condition 都要求 lio != cartographer）→ 不会出现两个 /map 发布者。
    start_cartographer_as_lio_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rm_nav_bringup_dir, 'launch', 'cartographer_sim.launch.py')),
        condition = carto_as_lio_mapping_condition,
        launch_arguments={
            'configuration_basename': 'cartographer_lio.lua',
            # 全包形态没有 LIO 的 /odom → 用仿真底盘里程计喂 use_odometry（实车换成下位机轮速）
            'odom_topic': '/odom_ground_truth'}.items()
    )

    # 纯建图（mode:=mapping）不再启动 nav2，因此 nav2 自带的 rviz_launch 也不会起。
    # 但建图时最需要的可视化恰恰是 RViz（看 /map 边建边长）→ 这里单独补一个 RViz。
    mapping_rviz_node = Node(
        condition = IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'mapping' and '",
            LaunchConfiguration('nav_rviz'), "' == 'True'"])),
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', os.path.join(get_package_share_directory('rm_navigation'), 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        # 只在 nav / slam_nav 下启动；纯建图(mapping)不需要规划控制，省 CPU（重场景 RTF 本来就紧张）
        condition = nav_stack_condition,
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': empty_map_dir,
            **nav2_params_launch_args,
            'global_obstacle': LaunchConfiguration('global_obstacle'),
            'local_obstacle': LaunchConfiguration('local_obstacle'),
            'nav_rviz': use_nav_rviz}.items()
    )

    # 在mapping模式下且不使用slam时，加载空地图
    # map_server现在由lifecycle_manager管理

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_spin_speed_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_LIO_cmd)
    ld.add_action(declare_nav_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_mapper_cmd)
    ld.add_action(declare_global_obstacle_cmd)
    ld.add_action(declare_local_obstacle_cmd)

    ld.add_action(start_rm_simulation)
    ld.add_action(bringup_imu_complementary_filter_node)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    ld.add_action(bringup_LIO_group)
    
    # T1：ICP 模式的帧桥（由条件控制，仅 nav+icp+LIO 时生效）
    ld.add_action(tf_bridge_node)
    ld.add_action(tf_bridge_node2)
    # lio:=small_point_lio 的回退帧桥（仅 nav + localization:='' + 该槽位时生效）
    ld.add_action(tf_bridge_spl_map_to_odom_node)
    
    # 启动时序：Gazebo 生成机器人 + LIO 初始化需要几秒，
    # 定位链延后 4s、Nav2 延后 10s，避免 costmap/amcl 在 odom/map 尚未出现时激活失败
    ld.add_action(TimerAction(period=4.0, actions=[start_localization_group]))
    ld.add_action(bringup_fake_vel_transform_node)
    ld.add_action(lio_tf_adapter_node)
    ld.add_action(TimerAction(period=4.0, actions=[start_mapping, start_cartographer_mapping,
                                                   start_cartographer_as_lio_mapping]))
    ld.add_action(TimerAction(period=10.0, actions=[start_navigation2]))
    # 纯建图模式的 RViz（nav2 未启动时 nav_rviz 仍要能出图）
    ld.add_action(mapping_rviz_node)

    return ld
