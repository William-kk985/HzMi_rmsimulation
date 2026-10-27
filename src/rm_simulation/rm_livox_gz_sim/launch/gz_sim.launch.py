# =============================================================================
# gz_sim.launch.py —— `sim_backend:=gz` 的 opt-in 入口（Gazebo Fortress）
# =============================================================================
# 它做四件事，别的一概不碰（感知/LIO/nav2 仍由 bringup_sim.launch.py 起，与 Classic 相同）：
#   1. `ign gazebo` 起 Fortress（世界 = 我们的 SDF 1.9；GUI 可选；HOME 自动重定向）；
#   2. ros_gz_bridge 桥 **/clock**（这是唯一必须桥的）；
#   3. robot_state_publisher + joint_state_publisher（TF 契约与 Classic 逐字相同）；
#   3b. **每次运行独占一个 gz transport partition**（GZ_PARTITION/IGN_PARTITION）。
#       不隔离的后果是实测过的（docs/fortress_sim_backend.md §8.2）：同机另一个
#       Fortress 世界会把自己的 /clock 与 /mid360/imu_gz 送进我们的插件
#       ⇒ /livox/imu 的戳变成**两个仿真钟交错** ⇒ small_point_lio 每秒每条都报
#       `imu loop back`、永不初始化（既无 odom→base_link 也无 /cloud_registered）。
#   4. 运行期把 robot 的 URDF 派生成 Fortress 的 SDF 模型（scripts/make_robot_gz_sdf.py），
#      并把 sensor 模型里的 @MID360_CSV@ 换成仓库里那份 17 MB 的 mid360.csv 真路径。
#
# 单独用（不开感知链）：
#   ros2 launch rm_livox_gz_sim gz_sim.launch.py world:=RMUC2026 gui:=False
#   ros2 launch rm_livox_gz_sim gz_sim.launch.py gz_world:=bench gz_robot:=none gui:=False
#
# 通过 bringup（推荐，整栈 + 端到端）：
#   ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \
#       world:=RMUC2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 \
#       gui:=False spin_speed:=0.0
#   （★ 2026-10-27：`robot11_mount` 已删除 —— 安装方式固定 = xacro 默认的 `sensor`。）
# =============================================================================
import os
import sys
import tempfile
import uuid
import shutil

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, AppendEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'rm_livox_gz_sim'

# world 槽位 -> 我们的 SDF（Classic 的 world 名到 gz 世界的映射，纯增量：未知值报错）
WORLD_MAP = {
    'RMUC2026': 'rmuc2026_gz.sdf',
    'RMUC2026_bench': 'rmuc2026_gz_bench.sdf',
}

# 由 tools/scripts/gzsim/fetch_and_patch_mid360.sh 编出来的后端前缀
GZSIM_WS_DEFAULT = os.path.join(os.path.expanduser('~'), 'gzsim_ws')
# 本仓的 scratch 约定（用户按 fetch 脚本的默认值编到仓库内时走这条）
REPO_GZSIM_WS = None  # 运行期由 _repo_root() 算


def _repo_root():
    """从本包的 install/share 路径反推仓库根（…/install/rm_livox_gz_sim/share/…）。"""
    here = os.path.dirname(os.path.realpath(__file__))
    # share/rm_livox_gz_sim/launch/gz_sim.launch.py -> 上溯 3 层到 install/
    p = here
    for _ in range(2):
        p = os.path.dirname(p)
    # p = <prefix>/share ; prefix = <prefix>
    prefix = os.path.dirname(p)
    if os.path.basename(prefix) == 'install':
        return os.path.dirname(prefix)
    return None


def _find_backend_prefix(log):
    """找 gz 后端（rmagine + rmagine_gazebo_plugins + mid360_simulation_plugin_ros2）前缀。

    顺序：GZSIM_PREFIX 环境变量 -> ament 索引 -> 仓库内 scratch 默认位置。
    找不到就抛一条可操作的错误（不是 Python traceback）。
    """
    cands = []
    env = os.environ.get('GZSIM_PREFIX')
    if env:
        cands.append(env)
    for pkg in ('mid360_simulation_plugin_ros2', 'rmagine_gazebo_plugins'):
        try:
            share = get_package_share_directory(pkg)
            # <prefix>/share/<pkg> -> <prefix>
            cands.append(os.path.dirname(os.path.dirname(share)))
        except PackageNotFoundError:
            pass
    root = _repo_root()
    if root:
        cands.append(os.path.join(root, '.tmp_gzlivox2', 'gzsim_ws', 'install'))
    cands.append(GZSIM_WS_DEFAULT)
    for c in cands:
        if os.path.isfile(os.path.join(c, 'mid360_simulation_plugin_ros2', 'lib',
                                       'libmid360_system.so')):
            log('gz 后端前缀 = %s' % c)
            return c
    raise RuntimeError(
        '找不到 Gazebo Fortress 的 MID-360 后端（libmid360_system.so）。\n'
        '  先跑一次（一次就够，源码不进本仓库）：\n'
        '      tools/scripts/gzsim/fetch_and_patch_mid360.sh\n'
        '  或把已有构建指过来：export GZSIM_PREFIX=/path/to/gzsim_ws/install\n'
        '  当前找过：\n    ' + '\n    '.join(cands))


def _render_model(share, out_models, csv, qos, ignore_self, log):
    """把 models/rm_mid360_gz 渲染到运行期目录（替换 @MID360_CSV@/@QOS@）。"""
    src = os.path.join(share, 'models', 'rm_mid360_gz')
    dst = os.path.join(out_models, 'rm_mid360_gz')
    os.makedirs(dst, exist_ok=True)
    text = open(os.path.join(src, 'model.sdf')).read()
    text = (text.replace('@MID360_CSV@', csv).replace('@QOS@', qos)
                .replace('@IGNORE_SELF@', ignore_self))
    for tok in ('@MID360_CSV@', '@QOS@', '@IGNORE_SELF@'):
        if tok in text:
            raise RuntimeError('%s 没替换干净' % tok)
    open(os.path.join(dst, 'model.sdf'), 'w').write(text)

    # 世界文件里也引用了 model://rm_mid360_gz（台架世界），运行期这份优先（在
    # GZ_SIM_RESOURCE_PATH 最前面）。
    open(os.path.join(dst, 'model.config'), 'w').write(
        '<?xml version="1.0" ?>\n<model>\n  <name>rm_mid360_gz</name>\n'
        '  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n'
        '  <description>runtime-rendered by rm_livox_gz_sim/launch/gz_sim.launch.py'
        '</description>\n</model>\n')
    log('sensor 模型渲染到 %s（csv=%s qos=%s ignore_self=%s）'
        % (dst, csv, qos, ignore_self))


def _materialize(context, *args, **kwargs):
    log_lines = []

    def log(msg):
        log_lines.append(msg)
        print('[gz_sim.launch] ' + msg, file=sys.stderr)

    share = get_package_share_directory(PKG)
    backend = _find_backend_prefix(log)

    world_sel = context.perform_substitution(LaunchConfiguration('gz_world'))
    world_arg = context.perform_substitution(LaunchConfiguration('world'))
    gui = context.perform_substitution(LaunchConfiguration('gui'))
    robot = context.perform_substitution(LaunchConfiguration('robot'))
    robot_desc = context.perform_substitution(LaunchConfiguration('robot_description'))
    qos = context.perform_substitution(LaunchConfiguration('gz_qos'))
    ignore_self = context.perform_substitution(LaunchConfiguration('gz_ignore_self'))
    verbose = context.perform_substitution(LaunchConfiguration('gz_verbose'))
    gz_home = context.perform_substitution(LaunchConfiguration('gz_home'))
    csv_override = context.perform_substitution(LaunchConfiguration('gz_csv'))
    runtime_dir = context.perform_substitution(LaunchConfiguration('gz_runtime_dir'))
    extra_guis = context.perform_substitution(LaunchConfiguration('gz_extra_gui'))
    partition = context.perform_substitution(LaunchConfiguration('gz_partition'))
    custom_msg_tag = context.perform_substitution(LaunchConfiguration('gz_custom_msg_tag'))
    drive = context.perform_substitution(LaunchConfiguration('gz_drive'))

    # ── 硬要求校验（缺了给可操作的报错，不静默跑错） ───────────────────────
    if robot not in ('', 'none', 'robot11'):
        raise RuntimeError(
            'sim_backend:=gz 目前只支持 robot:=robot11（其余槽位在 gz 侧没有 Fortress 化的'
            '模型生成路径）。收到 robot=%r。' % robot)
    # ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已删除。安装方式只剩一种（= 原来的
    #   `sensor` 语义）：URDF 关节 rpy=0 + 倾角在 `livox_frame→imu_link` + 插件把点旋进
    #   水平的 `livox_frame`；gz 侧传感器 link pose 仍由下面的 `--tilt-rpy` 钉成 −0.5236。
    #   ⇒ 这里不再校验 mount（它没有第二个取值可校验了）。

    # ── 世界文件 ─────────────────────────────────────────────────────────────
    if world_sel == 'bench' or world_arg not in WORLD_MAP:
        if world_sel == 'bench':
            world_file = os.path.join(share, 'worlds', 'rmuc2026_gz_bench.sdf')
        else:
            raise RuntimeError(
                'sim_backend:=gz 目前只支持 world:=RMUC2026（可选 gz_world:=bench '
                '起台架世界）。收到 world=%r；Classic 的其它场地在 gz 侧还没有 SDF 1.9 '
                '资产 —— 加一份即可，见 docs/fortress_sim_backend.md「会坏的地方」。'
                % world_arg)
    else:
        world_file = os.path.join(share, 'worlds', WORLD_MAP[world_arg])

    # ── 运行期目录 ───────────────────────────────────────────────────────────
    if not runtime_dir:
        runtime_dir = os.path.join(tempfile.gettempdir(),
                                   'rm_livox_gz_sim_%s' % uuid.uuid4().hex[:8])
    models_dir = os.path.join(runtime_dir, 'models')
    os.makedirs(models_dir, exist_ok=True)

    # ── HOME 重定向（这条是硬坑：~/.ignition 只读会让 GUI abort 并 SIGKILL server）
    if not gz_home:
        gz_home = os.path.join(runtime_dir, 'home')
    for sub in ('', '.ignition', '.cache', '.config', '.local/share', '.runtime'):
        os.makedirs(os.path.join(gz_home, sub), exist_ok=True)
    os.chmod(os.path.join(gz_home, '.runtime'), 0o700)
    log('HOME（仅 gz 进程）= %s' % gz_home)

    # ── CSV ─────────────────────────────────────────────────────────────────
    if csv_override:
        csv = csv_override
    else:
        # 真源 = Classic 插件用的那一份（不复制、不提交副本）
        try:
            livox_share = get_package_share_directory('ros2_livox_simulation')
            csv = os.path.join(livox_share, 'scan_mode', 'mid360.csv')
        except PackageNotFoundError:
            csv = ''
        if not csv or not os.path.isfile(csv):
            raise RuntimeError(
                '找不到 mid360.csv（Classic 插件那份）。先 colcon build 本仓并 source '
                'install/setup.bash，或用参数 gz_csv:=/abs/path/mid360.csv 指定。')
    log('mid360.csv = %s' % csv)

    _render_model(share, models_dir, csv, qos, ignore_self, log)

    # ── 机器人模型（从 robot_description 派生） ──────────────────────────────
    actions_extra_env = {}
    if robot not in ('', 'none'):
        if not robot_desc.strip():
            # 单独跑本 launch（不经 bringup）时自己跑一次 xacro：robot11 只有这一个模型文件。
            # ★ 2026-10-27（契约 §4.15）：**不再传 `livox_mount`** —— 安装方式只剩一种，且已烘进
            #   生成物 xacro（`<xacro:arg name="livox_mount" default="sensor"/>` + 常量 property）
            #   ⇒ 取默认值 = 与原来传 `livox_mount:=sensor|urdf` 逐字节相同的生成物。
            # ⚠️ 安装倾角**不在这里传**：它是生成物 xacro 的 `livox_tilt_rpy` 默认值，源头 =
            #   上游 URDF 的 body_to_livox.rpy（契约 §4.12；measurements §31）。这里再写一个字面值
            #   就是第二个所有者，上游换支架后这条独立路径会偷偷用旧角度。
            import subprocess
            try:
                urdf_dir = os.path.join(get_package_share_directory('rm_nav_bringup'), 'urdf')
            except PackageNotFoundError:
                raise RuntimeError(
                    'sim_backend:=gz 需要 robot_description（bringup 会传），且本机没装 '
                    'rm_nav_bringup ⇒ 无法自己跑 xacro。\n'
                    '  单独跑本 launch 时：加 robot:=none（只起台架世界）。')
            xacro_file = os.path.join(urdf_dir, 'sentry_robot_robot11_sim.xacro')
            cmd = ['xacro', xacro_file]
            log('robot_description 为空 ⇒ 自己跑：%s' % ' '.join(cmd))
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                raise RuntimeError('xacro 失败：\n' + (r.stderr or r.stdout)[-3000:])
            robot_desc = r.stdout
        gen = os.path.join(share, 'scripts', 'make_robot_gz_sdf.py')
        import subprocess
        model_name = 'rm_%s_gz' % (robot or 'robot')
        cmd = [sys.executable, gen, '--urdf', '-', '--out-dir', models_dir,
               '--model-name', model_name, '--csv', csv, '--qos', qos,
               '--imu-frame', 'imu_link', '--ignore-self', ignore_self,
               '--custom-msg-tag', custom_msg_tag, '--drive', drive]
        # 30° 斜装：robot11 的生成物 xacro 已把 roll 烘进 `livox_frame→imu_link` 与插件
        # `<tilt_rpy>`（`base_link→livox_frame` rpy=0）。这里再显式钉一次，保证「不管 URDF
        # 怎么写，gz 侧的传感器 link pose 一定是 -0.5236」（B① 决策）。
        if robot == 'robot11':
            cmd += ['--tilt-rpy', os.environ.get('GZ_TILT_RPY', '-0.523598775598293 0 0')]
        log('生成 %s 的 Fortress SDF：%s' % (robot, ' '.join(cmd)))
        r = subprocess.run(cmd, input=robot_desc, capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError('make_robot_gz_sdf.py 失败：\n' + (r.stderr or r.stdout)[-3000:])
        for line in (r.stderr or '').strip().splitlines():
            log('  ' + line)
    else:
        model_name = None
        log('robot:=none ⇒ 不起机器人（台架世界，只有传感器）')

    # ── gz transport partition（**必须每次运行唯一**，理由见文件头 3b） ──────
    # 空 = 本 run 随机一个（默认，最安全）；指定 = 固定名（只有"故意要让两个世界
    # 共享 gz 话题"的对照实验才这么干）。
    if not partition:
        partition = 'rm_gz_%s' % uuid.uuid4().hex[:10]
    log('gz transport partition = %s（GZ_PARTITION / IGN_PARTITION 同时设）' % partition)

    # ── 环境变量（Fortress 两个世代的名字都设：本机实测两个都设才稳，见
    #    livox_sensor_prior_art.md §10.7） ────────────────────────────────────
    res_paths = [models_dir]
    # 场地 / 障碍物 / robot11 的 model:// 解析根（Classic 靠 GAZEBO_MODEL_PATH，这里显式给）
    try:
        res_paths.append(os.path.join(get_package_share_directory('hzmi_rm_simulation'),
                                      'meshes'))
    except PackageNotFoundError:
        pass
    try:
        res_paths.append(os.path.dirname(get_package_share_directory('robot11')))
    except PackageNotFoundError:
        pass
    res_paths.append(os.path.join(share, 'models'))
    plugin_paths = [
        os.path.join(backend, 'rmagine_gazebo_plugins', 'lib', 'rmagine_gazebo_plugins'),
        os.path.join(backend, 'mid360_simulation_plugin_ros2', 'lib'),
        # 上游装出来的 rpath 目录（rmagine 本体在这里）
        os.path.join(backend, 'rmagine', 'lib'),
    ]
    # 我们自己的底盘驱动插件（本仓包 rm_gz_planar_move，`<prefix>/lib/librm_gz_planar_move.so`）
    try:
        _planar_share = get_package_share_directory('rm_gz_planar_move')
        plugin_paths.append(os.path.dirname(os.path.dirname(_planar_share)))
        # 上面那一层是 <prefix>（share/rm_gz_planar_move → <prefix>），.so 在 <prefix>/lib
        plugin_paths[-1] = os.path.join(plugin_paths[-1], 'lib')
    except PackageNotFoundError:
        log('⚠ 没装 rm_gz_planar_move ⇒ gz 侧没有底盘驱动（/cmd_vel_chassis 无人消费）')
    for k in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'IGN_FILE_PATH'):
        actions_extra_env.setdefault(k, os.pathsep.join(res_paths))
    for k in ('GZ_SIM_SYSTEM_PLUGIN_PATH', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH'):
        actions_extra_env[k] = os.pathsep.join(plugin_paths)
    log('GZ_SIM_SYSTEM_PLUGIN_PATH = %s' % actions_extra_env['GZ_SIM_SYSTEM_PLUGIN_PATH'])
    # 两个拼法都设：ignition-transport11（Fortress）读 IGN_PARTITION，gz-transport12+
    # 读 GZ_PARTITION；与 GZ_SIM_SYSTEM_PLUGIN_PATH 同一套"两代都设"的策略。
    actions_extra_env['GZ_PARTITION'] = partition
    actions_extra_env['IGN_PARTITION'] = partition

    ld = [LogInfo(msg='[gz_sim.launch] ' + m) for m in
          ('world = %s' % world_file, 'runtime dir = %s' % runtime_dir)]
    for k in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'IGN_FILE_PATH',
              'GZ_SIM_SYSTEM_PLUGIN_PATH', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
              'GZ_PARTITION', 'IGN_PARTITION'):
        ld.append(SetEnvironmentVariable(k, actions_extra_env[k]))

    # ── ign gazebo ───────────────────────────────────────────────────────────
    cmd = ['ign', 'gazebo', '-r', '-v', '3']
    if gui not in ('True', 'true', '1'):
        cmd.append('-s')            # 无 GUI（server only）
    cmd.append(world_file)
    gz_env = {
        'HOME': gz_home,
        'XDG_CACHE_HOME': os.path.join(gz_home, '.cache'),
        'XDG_CONFIG_HOME': os.path.join(gz_home, '.config'),
        'XDG_DATA_HOME': os.path.join(gz_home, '.local', 'share'),
        'XDG_RUNTIME_DIR': os.path.join(gz_home, '.runtime'),
        'LIBGL_ALWAYS_SOFTWARE': '1',      # 无 GPU 机器：llvmpipe
    }
    log('gz 命令：' + ' '.join(cmd))
    ld.append(ExecuteProcess(cmd=cmd, output='screen', additional_env=gz_env))

    # ── /clock 桥（唯一必须桥的东西） ─────────────────────────────────────────
    bridge_cfg = os.path.join(share, 'config', 'ros_gz_bridge.yaml')
    ld.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                   name='gz_clock_bridge', output='screen',
                   arguments=['--ros-args', '-p',
                              'config_file:=%s' % bridge_cfg]))

    # ── TF：与 Classic 逐字相同的两个发布者 ─────────────────────────────────
    if robot not in ('', 'none'):
        use_sim_time = LaunchConfiguration('use_sim_time')
        ld.append(Node(package='joint_state_publisher', executable='joint_state_publisher',
                       name='joint_state_publisher', output='screen',
                       parameters=[{'use_sim_time': use_sim_time},
                                   {'robot_description': robot_desc}]))
        ld.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                       name='robot_state_publisher', output='screen',
                       parameters=[{'use_sim_time': use_sim_time},
                                   {'robot_description': robot_desc}]))
    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='RMUC2026',
                              description='与 Classic 同名；gz 侧目前只有 RMUC2026'),
        DeclareLaunchArgument('gz_world', default_value='',
                              description="'' = 用 world 槽位；'bench' = 台架世界（只有传感器，"
                                          '没有机器人；docs 里所有契约/性能数字都是它跑出来的）'),
        DeclareLaunchArgument('robot', default_value='',
                              description="''/none = 不起机器人；robot11 = 从 robot_description "
                                          '派生 Fortress 的 SDF 模型'),
        # ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已删除（安装方式固定 = xacro 默认的
        #   `sensor`；本 launch 不再声明/读取它）。
        DeclareLaunchArgument('robot_description', default_value=''),
        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('gz_qos', default_value='best_effort',
                              description='点云发布 QoS：best_effort（默认，SensorDataQoS）|'
                                          'reliable（只用于复现"慢消费者堵死 gz 线程"）'),
        DeclareLaunchArgument('gz_verbose', default_value='3'),
        DeclareLaunchArgument('gz_ignore_self', default_value='true',
                              description='true（默认）= 自机排除生效（robot 不进 BVH）；'
                                          'false = **只用于对照实验**，让传感器看得见自己的车体'),
        DeclareLaunchArgument('gz_home', default_value='',
                              description='仅 gz 进程的 HOME。空 = 运行期临时目录'
                                          '（本机 ~/.ignition 只读会让 GUI abort）'),
        DeclareLaunchArgument('gz_csv', default_value='',
                              description='空 = 用 ros2_livox_simulation 包里那份 mid360.csv'),
        DeclareLaunchArgument('gz_runtime_dir', default_value='',
                              description='空 = /tmp/rm_livox_gz_sim_<rand>；调试时指定可回看生成物'),
        DeclareLaunchArgument('gz_extra_gui', default_value=''),
        DeclareLaunchArgument('gz_drive', default_value='true',
                              description='true（默认）= 给 gz 机器人加**运动学底盘驱动**'
                                          '（/cmd_vel_chassis → 位姿 + /odom_ground_truth，'
                                          'Classic 的 planar_move 的对应实现）；false = 静态底座'
                                          '（A/B 对照用）'),
        DeclareLaunchArgument('gz_custom_msg_tag', default_value='0',
                              description='/livox/lidar（CustomMsg）的每点 tag。0（默认）= 与 '
                                          'Classic 逐字节一致；16 = 上游写死的 0x10（**只用于'
                                          '复现 small_point_lio 全丢点**）'),
        DeclareLaunchArgument('gz_partition', default_value='',
                              description='gz transport partition（GZ_PARTITION/IGN_PARTITION）。'
                                          '空 = 本次运行随机一个（**默认，必须**：同机另一个 '
                                          'Fortress 世界会把 /clock 与 /mid360/imu_gz 送进来，'
                                          '导致 /livox/imu 戳交错、small_point_lio 刷 '
                                          '`imu loop back`）；只有"故意共享 gz 话题"的对照实验'
                                          '才显式给一个名字'),
        OpaqueFunction(function=_materialize),
    ])
