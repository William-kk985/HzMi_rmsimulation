import os
import sys
import uuid
import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, LogInfo,
                            TimerAction, OpaqueFunction, RegisterEventHandler, Shutdown,
                            AppendEnvironmentVariable)
from launch.event_handlers import OnProcessExit
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
    'patchwork_ground_segmentation': 'ground:=linefit（默认值，现有已验证路径）',
    'rm_ground_traversability': '无替代槽位：它是两个地面分割节点的**编译期依赖**（header-only 判据库），'
                                '不构建它两个节点都编不过 —— 见 docs/ground_segmentation_slots.md §10',
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


# =============================================================================
# robot:=<模型> 槽位（2026-10-07 新增）——「换机器人模型试一试」的**唯一开关**
# -----------------------------------------------------------------------------
# 为什么要有它：用户给了一份别人的哨兵 URDF（hzmirmvision-master/configs/sentry_robot.urdf，
# 里面雷达装在云台头上），想在本仿真里"试一试"。但那份 URDF 是**纯描述文件**：
# 没有 inertial（Gazebo 会把无惯性的 link 整条丢掉 ⇒ spawn 出来是空模型）、
# 没有 <gazebo>/<sensor>/<plugin>、没有 imu_link ⇒ 直接换上去本栈一个话题都不会有。
# 所以做法是：**新增一个 opt-in 槽位**，默认值仍然是现在的模型（一个字节都不变），
# 只有显式 `robot:=hzmirm` 才切到"他们的运动学 + 我们补的仿真件"。
#
# 设计要点（与文件开头那段 _PackageShareFile 的原则一致）：
#   · 全部惰性：不选这个槽位时，这两个 Substitution 的 perform() 根本不会被调用
#     ⇒ 路径不存在/文件没构建也不会挡住别的组合，`--show-args` 同样不受影响；
#   · 默认路径逐字节不变：robot 留空时 _RobotXacroCommand 返回的命令字符串
#     与改造前 Command([...]) 拼接出来的**完全相同**；
#   · 选错值 → 抛一条可操作的 RuntimeError（列出可用取值），launch 正常收尾退出码 1。
# =============================================================================

#: 可用的 robot 槽位值 → (相对 urdf/ 的文件名, 人话说明)
_ROBOT_SLOTS = {
    '': ('sentry_robot_sim.xacro', '默认模型（本仓现行，雷达在 base_link+0.12,0,0.175）'),
    'hzmirm': ('sentry_robot_hzmirm_sim.xacro',
               '用户给的哨兵 URDF（雷达在云台头上 base_link+0,0,0.8；运动学逐字保留）'),
    'robot11': ('sentry_robot_robot11_sim.xacro',
                '用户的哨兵 robot11（SolidWorks 导出：真 mesh + 真 inertial；'
                '雷达在底盘上、斜 30° 下俯、离地 0.2595 m）'),
}

#: robot:=robot11 的雷达 30° 安装：**上游两份材料自相矛盾**（URDF 原文写 pitch、
#: SolidWorks CSV 写 roll），Phase 2 只能证明"轴看得出来、哪个对文件说了算"。
#: ★ 2026-10-07 Phase 3：**用户按实物确认 = 绕 roll**（"30° 是绕 roll 轴"）⇒ 默认 roll 从
#:   "机器导出的 CSV 更可信"升级为"实物判据"；`pitch` 保留为对照开关（几何上等价，只差绕 z 一转）。
_LIVOX_TILT_RPY = {
    # 默认：SolidWorks CSV 的 Joint Origin Roll = -0.523598775598293（**用户已按实物确认**）
    'roll': '-0.523598775598293 0 0',
    # URDF 原文那一行 rpy="0 -0.5236 0"（保留供对照/回退）
    'pitch': '0 -0.523598775598293 0',
}

#: robot:=robot11 的 A 方案（碰撞挖视锥）默认抬高量：**0 = 与上游几何逐字相同**。
#: 非 0 = B 方案（把雷达抬到顶板上方）—— 只是"实测对照"用；抬高会改变雷达离地高度，
#: linefit 的 sensor_height 必须同步改（见 config/segmentation_sim_robot11.yaml 的 ⚠️ 与
#: docs/robot_models.md §11 的 B 复现步骤）。本 launch **不替你改 YAML**（避免第二个真源）。
_LIVOX_RAISE_DEFAULT = '0.0'

#: ★ 2026-10-08：robot:=robot11 的**雷达安装方式**（30° 倾角记在哪个坐标系上）。
#:   · 'plugin'（**默认 = 2026-10-07 起的行为，逐字节不变**）：`body_to_livox` 关节 rpy = 0
#:     （`livox_frame` 帧**重力对齐**），倾角由插件 `<tilt_rpy>` 承担 ⇒ **射线真的斜 30°**、
#:     点云表达在水平的父 link 系、`/scan` 的高度带与 linefit 的 sensor_height 语义都对；
#:     代价：TF 里帧是平的、**画出来的雷达 mesh 也是平的**（用户看到的"URDF/雷达图像是平放的"）。
#:   · 'urdf'：倾角放回 URDF 关节（rpy = −0.5236 roll = 上游/CSV 的字面值 = **实物的物理安装
#:     姿态**，连 mesh 一起斜），插件 `<tilt_rpy>` = 单位阵 ⇒ 世界里的射线方向与 plugin 档**逐条
#:     相同**，但点云表达在**斜的传感器系**里（地面在点云里是 30° 斜面）。
#:     ⚠️ 本档**不动任何感知参数** —— 就是要看"物理安装保真"的原始后果（哪些语义会跟着变，
#:     见 docs/tilted_lidar_fidelity.md §C）。
_LIVOX_MOUNTS = ('plugin', 'urdf')


def _validate_livox_mount(value):
    """把 `robot11_mount` 的取值验成 `plugin|urdf`（惰性；选错给可操作报错，不静默回退）。"""
    v = (value or '').strip() or 'plugin'
    if v not in _LIVOX_MOUNTS:
        raise RuntimeError('[launch] robot11_mount:=%r 不是可用取值（%s）'
                           % (value, ' | '.join(_LIVOX_MOUNTS)))
    return v


def _rpy_deg_to_R(rpy_rad):
    """固定关节 rpy（弧度，URDF 的 Rz·Ry·Rx 约定）→ 旋转矩阵（行主序 3x3）。"""
    import math as _m
    r, p, y = rpy_rad
    cr, sr = _m.cos(r), _m.sin(r)
    cp, sp = _m.cos(p), _m.sin(p)
    cy, sy = _m.cos(y), _m.sin(y)
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return [[sum(Rz[i][k] * sum(Ry[k][j] * Rx[j][m] for j in range(3))
                 for k in range(3)) for m in range(3)] for i in range(3)]


def _lio_adapter_robot11(axis, raise_m=0.0, mount='plugin'):
    """robot:=robot11 的 lio_tf_adapter 补偿量（xyz = base_link 原点在 imu 系下的坐标，rpy = 其姿态）。

    纯几何、无魔法：imu_link 的位姿 = body_to_livox(rpy) ∘ (0,0,-0.05)（imu_joint 是 rpy=0 的
    固定关节 ⇒ **IMU 跟着雷达一起动**，这正是让三份 LIO 配置的 extrinsic_T=[0,0,0.05]
    一个字节都不用改的原因）。T_imu←base_link 就是它的逆。

    ★ 2026-10-08：`mount` 决定 `body_to_livox` 的 rpy 取哪个值（与 xacro 里的同一个规则）：
      · `plugin`（默认）：rpy = 0 ⇒ 与 Phase 3 的返回值**逐位相同**（旋转为单位阵、xyz = −p_imu）；
      · `urdf`          ：rpy = 倾角（roll/pitch 30°）⇒ **旋转不再是单位阵**，xyz 也要用 Rᵀ 转一次。
    两种挂法下 imu 的**离地高度**都还是 0.216226 m（30° 只改朝向、不改高度？—— 不：见 §C 实测，
    `urdf` 档 imu 随雷达一起斜 ⇒ 离地高度由 R 决定；本函数按几何**算**出来，不写死）。
    """
    import math as _m
    tilt = [float(v) for v in _LIVOX_TILT_RPY[axis].split()]
    mount = (mount or 'plugin').strip() or 'plugin'
    rpy = [0.0, 0.0, 0.0] if mount == 'plugin' else tilt
    p_livox = (0.000561701465058485, 0.130915824456595, 0.15702816968305 + float(raise_m))
    R = _rpy_deg_to_R(rpy)                      # R_body←livox
    # imu 原点在 body 系 = p_livox + R·(0,0,−0.05)；姿态 = R
    off = [R[i][2] * -0.05 for i in range(3)]
    p_imu = [p_livox[i] + off[i] for i in range(3)]
    # T_imu←base_link：xyz = −Rᵀ·p_imu，rpy = R 的逆（= 同一个轴的负角）
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(Rt[i][j] * p_imu[j] for j in range(3)) for i in range(3)]
    rpy_out = [-v for v in rpy]
    return ([round(v, 9) for v in t], [round(v, 9) for v in rpy_out])


class _LioAdapterRobot11(Substitution):
    """把 robot11 的杆臂算成一段 **YAML 列表字符串**（launch_ros 会对 Substitution 的求值结果
    做 yaml.safe_load ⇒ 正好还原成 double 数组；见 launch_ros/utilities/evaluate_parameters.py）。

    惰性：只有 `robot:=robot11` 且 LIO 不是 none/cartographer/small_point_lio 时才会被求值。
    """

    def __init__(self, which):
        super().__init__()
        self.__which = which

    def perform(self, context):
        axis = LaunchConfiguration('livox_tilt_axis').perform(context).strip()
        if axis not in _LIVOX_TILT_RPY:
            raise RuntimeError("[launch] livox_tilt_axis:=%r 不是可用取值（roll | pitch）" % axis)
        raise_m = LaunchConfiguration('livox_raise_m').perform(context).strip() or '0.0'
        mount = _validate_livox_mount(LaunchConfiguration('robot11_mount').perform(context))
        xyz, rpy = _lio_adapter_robot11(axis, raise_m, mount)
        v = xyz if self.__which == 'xyz' else rpy
        return '[%s]' % ', '.join(repr(float(x)) for x in v)

    def describe(self):
        return '%s(%s)' % (type(self).__name__, self.__which)


class _Nav2ParamsForSlot(Substitution):
    """nav2 的**公共段**参数文件：`robot:=robot11` 时把槽位覆盖层**深合并**进公共段。

    为什么用"运行时合并"而不是多加一份参数文件：`bringup_rm_navigation.py` 只认三个槽
    （params_file / planner / controller），三份都被占满了；而 `src/rm_navigation/**/params`
    是别的任务的目录（本主题禁改）。⇒ 在**我们自己的 launch** 里把
    `rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml`（只含几何键）深合并到公共段之上，
    写成一份临时 YAML 再传下去。

    · 非 robot11 槽位：**原样返回公共段路径**（不合并、不写临时文件 ⇒ 行为与以前逐字节一致）；
    · robot11：合并（覆盖层的键赢），临时文件落在 `$ROS_LOG_DIR`（拿不到就 /tmp）。
    · 合并是**深**的（只覆盖叶子键）⇒ 公共段里其它一切（含注释以外的全部键）保持不变。
    """

    def __init__(self, base_path, override_path):
        super().__init__()
        self.__base = base_path
        self.__override = override_path

    def perform(self, context):
        import tempfile

        import yaml as _yaml
        if LaunchConfiguration('robot').perform(context).strip() != 'robot11':
            return self.__base
        if not os.path.isfile(self.__override):
            raise RuntimeError('[launch] robot:=robot11 的 nav2 覆盖层不在：%s' % self.__override)
        with open(self.__base) as f:
            base = _yaml.safe_load(f)
        with open(self.__override) as f:
            over = _yaml.safe_load(f)

        def deep_merge(dst, src):
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dst.get(k), dict):
                    deep_merge(dst[k], v)
                else:
                    dst[k] = v

        deep_merge(base, over)
        out_dir = os.environ.get('ROS_LOG_DIR') or tempfile.gettempdir()
        out = os.path.join(out_dir, 'nav2_params_sim_robot11_merged.yaml')
        with open(out, 'w') as f:
            _yaml.safe_dump(base, f, allow_unicode=True, sort_keys=False)
        return out

    def describe(self):
        return '%s(base=%s, override=%s)' % (type(self).__name__, self.__base, self.__override)


class _SlotSpinSpeedDefault(Substitution):
    """`spin_speed` 的**按槽位**默认值：robot:=robot11 ⇒ 0.0，其余 ⇒ 5.0（旧默认）。

    为什么不用裸 PythonExpression：它的求值时机是"声明这条 launch 参数时"，而 `robot` 这条
    参数如果声明得更晚（或来自命令行），那个时刻可能还不在 context 里 ⇒ PythonExpression 会抛
    `SubstitutionFailure`（实测：**整条 launch 起不来**，连默认模型都受影响）。
    这里显式容错：
      · `robot` 已在 context 里（= 声明顺序在前，或命令行给了）⇒ 按槽位给值；
      · 还没有 ⇒ 退回旧默认 5.0（不抛异常；用户仍可显式 `spin_speed:=0.0`）。
    另外 launch 里已把 `robot` 的声明**挪到 spin_speed 之前**，所以正常情况下走第一条分支。
    """

    def perform(self, context):
        try:
            slot = context.launch_configurations.get('robot', '')
        except Exception:                                             # noqa: BLE001
            slot = ''
        return '0.0' if str(slot).strip() == 'robot11' else '5.0'

    def describe(self):
        return '%s()' % (type(self).__name__)


def _robot_slot_error(value):
    return RuntimeError(
        "[launch] robot:=%r 不是可用的机器人模型槽位。\n"
        '  可用取值：\n%s\n'
        '  · 留空（默认）＝ 本仓现行模型，行为与以前完全一致；\n'
        '  · robot:=hzmirm  ＝ 用户给的哨兵 URDF 变体（详见 docs/robot_models.md §1–§8）；\n'
        '  · robot:=robot11 ＝ 用户的哨兵 robot11（真 mesh + 真 inertial；§9–§10）。'
        % (value, '\n'.join('      robot:=%-8s → %s' % (k or "''", v[1])
                            for k, v in sorted(_ROBOT_SLOTS.items()))))


class _RobotSlot(Substitution):
    """把 robot 槽位值解析成 urdf/ 下的 xacro 文件名（惰性，选错值给可操作报错）。"""

    def __init__(self, urdf_dir):
        super().__init__()
        self.__urdf_dir = urdf_dir

    def perform(self, context):
        slot = LaunchConfiguration('robot').perform(context).strip()
        if slot not in _ROBOT_SLOTS:
            raise _robot_slot_error(slot)
        return os.path.join(self.__urdf_dir, _ROBOT_SLOTS[slot][0])

    def describe(self):
        return '%s(urdf_dir=%s)' % (type(self).__name__, self.__urdf_dir)


class _RobotXacroCommand(Substitution):
    """拼出 `xacro <模型> xyz:=… rpy:=… [云台角]` 这条命令（惰性）。

    默认槽位的输出与改造前逐字节相同；只有 hzmirm 槽位才追加 turret_yaw_deg / turret_pitch_deg
    （默认 0 ⇒ 与上游 URDF 数值等价；非 0 用于试"雷达斜放/云台转动"假设，见 docs/robot_models.md §3）。
    """

    def __init__(self, urdf_dir, xyz, rpy):
        super().__init__()
        self.__slot = _RobotSlot(urdf_dir)
        self.__xyz = xyz
        self.__rpy = rpy

    def perform(self, context):
        cmd = ['xacro ', self.__slot.perform(context), ' xyz:=', self.__xyz, ' rpy:=', self.__rpy]
        slot = LaunchConfiguration('robot').perform(context).strip()
        if slot == 'hzmirm':
            cmd += [' turret_yaw_deg:=', LaunchConfiguration('turret_yaw_deg').perform(context),
                    ' turret_pitch_deg:=', LaunchConfiguration('turret_pitch_deg').perform(context)]
        elif slot == 'robot11':
            # ★ 2026-10-07：robot11 的雷达 30° 安装轴（roll|pitch，上游两份材料矛盾 ⇒ 不猜）。
            #   ⚠️ 值里带空格 ⇒ **必须像 xyz/rpy 那样加引号**，否则 xacro 会把它当成多个输入文件
            #   （实测报 `xacro: error: expected exactly one input file as argument`）。
            cmd += [' livox_tilt_rpy:="',
                    _LIVOX_TILT_RPY[LaunchConfiguration('livox_tilt_axis').perform(context).strip()],
                    '"']
            # ★ 2026-10-08：**安装倾角记在哪儿**（plugin = 今天的行为 / urdf = 实物的物理安装）。
            #   ⚠️ 与 livox_tilt_rpy 不同，这个值**不带空格**，不需要引号。
            cmd += [' livox_mount:=',
                    _validate_livox_mount(
                        LaunchConfiguration('robot11_mount').perform(context))]
            # ★ Phase 3：B 方案（抬高雷达）的开关。默认 0 ⇒ 与上游几何逐字相同。
            cmd += [' livox_raise_m:=',
                    LaunchConfiguration('livox_raise_m').perform(context).strip() or '0.0']
            # ★ Phase 4 新增 / **Phase 5 改指向**：视觉 mesh 档位（`decimated` | `full`）。
            #   只影响 `<visual>`，不影响任何 `<collision>`/传感器/插件 ⇒ 物理与契约不变。
            #   Phase 5 起 `decimated` 指的是**专用视觉 LOD**（`meshes/decimated/`，整车 163,995 面），
            #   不再是 Phase 4 借用的**碰撞档**抽稀件（`meshes/generated/`，整车 9,700 面 ——
            #   `l11` 只有 1,200 面，拿它当视觉会把云台/发射机构糊成一团）。
            #   两个 arg 的优先级：`visual_decimated` 非空时赢（= 命令行优先），都为空 ⇒ decimated。
            cmd += [' visual_decimated:=',
                    'true' if LaunchConfiguration('robot11_visual').perform(
                        context).strip() == 'decimated' else 'false']
        return ''.join(cmd)

    def describe(self):
        return '%s(%s)' % (type(self).__name__, self.__slot.describe())


class _RobotSlotFile(Substitution):
    """按 robot 槽位选一个"包 share 下的参数文件"（惰性；缺包/缺文件给可操作报错）。

    与 _PackageShareFile 的区别只有一个：文件名随 robot 槽位变。
    默认槽位返回的路径与改造前**逐字节相同** ⇒ 默认模型读的还是同一份 YAML。
    """

    def __init__(self, package_name, default_rel, alt_rel=None, slot='hzmirm',
                 error_hint='', slot_map=None):
        super().__init__()
        self.__package_name = package_name
        self.__default_rel = tuple(default_rel)
        #: 槽位 → 该槽位要用的文件（`alt_rel`/`slot` 是单槽位的旧用法，保留不动）
        self.__slot_map = {k: tuple(v) for k, v in (slot_map or {}).items()}
        if alt_rel is not None:
            self.__slot_map[slot] = tuple(alt_rel)
        self.__error_hint = error_hint

    def perform(self, context):
        rel = self.__slot_map.get(
            LaunchConfiguration('robot').perform(context).strip(), self.__default_rel)
        try:
            share_dir = get_package_share_directory(self.__package_name)
        except PackageNotFoundError:
            raise RuntimeError("[launch] robot 槽位需要 ROS 包 '%s'，但当前 AMENT_PREFIX_PATH 里没有"
                               '（没构建过，或构建后没有重新 source install/setup.bash）。%s'
                               % (self.__package_name, ('\n  · ' + self.__error_hint)
                                  if self.__error_hint else ''))
        path = os.path.join(share_dir, *rel)
        if not os.path.isfile(path):
            raise RuntimeError('[launch] robot 槽位要的参数文件不在：%s%s'
                               % (path, ('\n  · ' + self.__error_hint) if self.__error_hint else ''))
        return path

    def describe(self):
        return '%s(package=%s, default=%s, slots=%s)' % (
            type(self).__name__, self.__package_name,
            os.path.join(*self.__default_rel),
            {k: os.path.join(*v) for k, v in self.__slot_map.items()})


class _YamlKeysReadout(Substitution):
    """把"参数文件路径"的 substitution 求值后，读出若干键的**生效值**打成一个字符串。

    ★ 2026-10-07 Phase 4（缺陷 ③ 的修法）：上一轮的横幅写死了"linefit 参数**没有**跟着切"，
    而代码里其实**已经切了**（`_RobotSlotFile` 的 slot_map）⇒ 横幅在骗人，用户照它排查就查错方向。
    根因是"横幅说的是**意图**，不是**生效值**"。这里改成：**运行时**把真正要递给节点的那份
    YAML 读出来、把键值打进日志 ⇒ 横幅与"节点实际读到的文件"不可能再分叉
    （真正的节点侧运行期证据仍是 `ros2 param get /ground_segmentation sensor_height`）。
    """

    def __init__(self, path_sub, keys):
        super().__init__()
        self.__sub = path_sub
        self.__keys = tuple(keys)

    def perform(self, context):
        path = self.__sub.perform(context)
        vals = {}
        for line in open(path, encoding='utf-8'):
            line = line.split('#')[0].rstrip()
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            k, v = k.strip(), v.strip()
            if k in self.__keys:
                vals[k] = v or '""'
        got = ', '.join('%s=%s' % (k, vals.get(k, '?')) for k in self.__keys)
        return '%s  ← %s' % (got, path)

    def describe(self):
        return '%s(keys=%s)' % (type(self).__name__, list(self.__keys))


class _MountReadout(Substitution):
    """robot:=robot11 的**安装方式**在横幅里的收据（运行时按 robot11_mount 给不同的话）。

    ★ 2026-10-08：横幅不能写死"livox_frame 的 rpy=0" —— `robot11_mount:=urdf` 时它是斜的。
    这里把"当前这一跑到底把倾角记在哪儿、哪些下游语义会跟着变"如实打出来。
    """

    def perform(self, context):
        m = _validate_livox_mount(LaunchConfiguration('robot11_mount').perform(context))
        if m == 'plugin':
            return ('  · 雷达安装方式 robot11_mount=plugin（默认，= 2026-10-07 起的行为）：'
                    '倾角记在插件 `<tilt_rpy>`；`livox_frame` 帧**重力对齐**（TF rpy=0）、'
                    '点云表达在水平的父 link 系 ⇒ linefit 的 sensor_height / p2l 的高度带 / '
                    '自击掩膜的 z 门限**语义都对**；代价 = TF 与画出来的雷达 mesh 是平的。'
                    'lio_tf_adapter 杆臂是**纯平移**（lio:=small_point_lio/cartographer/none 时'
                    '该节点不启动，由 LIO 用 TF 自己做相似变换）')
        return ('  · 雷达安装方式 robot11_mount=urdf（**物理安装保真档**）：倾角写在 URDF 关节'
                '（rpy = −0.5236 roll，上游/CSV 字面值，**画的 mesh 一起斜**）、插件 `<tilt_rpy>` = 单位阵。'
                '世界里的射线方向与 plugin 档**逐条相同**，但点云表达在**斜的传感器系**（地面在点云里是 '
                '30° 斜面）⇒ linefit 的 sensor_height / p2l 的 min/max_height / 自击掩膜的 z 门限'
                '**都还在按"点云是重力对齐"解释**（本档**故意不调**，就是要看原始效果）；'
                'lio_tf_adapter 杆臂**带 30° 旋转**（已按几何算，不再是纯平移）')

    def describe(self):
        return '%s()' % type(self).__name__


class _SelfMaskReadout(Substitution):
    """自击掩膜参数文件的**生效值**：`enable` + 盒子个数（同一份文件，节点读的就是它）。"""

    def __init__(self, path_sub):
        super().__init__()
        self.__sub = path_sub

    def perform(self, context):
        path = self.__sub.perform(context)
        enable, boxes, radius, zmin = '?', 0, 0.0, None
        acc = None                 # 点表可能跨多行（生成器按 ~98 列折行）⇒ 攒到 ']' 为止
        for raw in open(path, encoding='utf-8'):
            line = raw.split('#')[0]
            if acc is None:
                s = line.strip()
                if s.startswith('self_mask_enable:'):
                    enable = s.split(':', 1)[1].strip()
                elif s.startswith('self_mask_radius_m:'):
                    radius = float(s.split(':', 1)[1])
                elif s.startswith('self_mask_z_min_m:'):
                    zmin = float(s.split(':', 1)[1])
                elif s.startswith('self_mask_boxes:'):
                    acc = s.split(':', 1)[1]
            else:
                acc += ' ' + line.strip()
            if acc is not None and ']' in acc:
                boxes = len([x for x in acc.strip().strip('[]').split(',') if x.strip()])
                acc = None
        return ('self_mask_enable=%s, 近场死区 r<=%.4f m%s, %d 个 collision AABB  ← %s'
                % (enable, radius,
                   ('（z>=%.4f）' % zmin) if zmin is not None else '',
                   int(boxes / 6), path))

    def describe(self):
        return '%s()' % type(self).__name__


# =============================================================================
# `model://` 的本地解析根（2026-10-08 新增）—— 让 `robot:=robot11` **不再需要手工前缀环境变量**
# -----------------------------------------------------------------------------
# 现象（用户 2026-10-07）：「gazebo加载不出来吗，rviz倒是挺好的看着」。
# 机理（取证见 docs/gazebo_gui_troubleshooting.md §3）：
#   ① URDF 里 12 个 `<visual>` 写的是 `package://robot11/meshes/decimated/<link>.stl`；
#   ② sdformat 做 URDF→SDF 时把它**改写成** `model://robot11/meshes/decimated/<link>.stl`；
#   ③ gazebo 的 `model://` 解析根 = `$HOME/.gazebo/models` + `GAZEBO_MODEL_PATH`
#      （后者由 gazebo_ros 的 gzserver/gzclient launch 扫描各包 package.xml 的
#       `<export><gazebo_ros gazebo_model_path=…/></export>` 拼出来）；
#   ④ 本地解析不到 ⇒ `SystemPaths::FindFileURI()` **无条件**回落到在线模型库
#      `ModelDatabase::GetModelPath(uri, /*forceDownload=*/true)` ⇒ `GetModels()` 抢不到后台
#      抓取线程的锁 ⇒ 打印 `Waiting for model database update to complete...` 并**同步阻塞**
#      （实测 stall 48.03 / 76.34 / 99.67 s，三次都是被人打断的，**不设上限**）；
#   ⑤ 网络那一路最后也失败 ⇒ `Visual.cc:2956 No mesh specified` ×24 ⇒ **车在 Gazebo 里没有视觉**
#      （RViz 不受影响：它走 `package://` + ament 索引，根本不经过 gazebo 的 `model://` 解析）。
#
# 修法（本段 = 「启动侧」，与 `package.xml` 的 `<export>` 互为兜底；两条都做，都是纯增量）：
#   把「本次 slot 真的用到的、带 mesh 的包」的 share **父目录**追加进 `GAZEBO_MODEL_PATH`
#   ——因为 `model://<pkg>/meshes/x.stl` 要求某个解析根 R 满足 `R/<pkg>/meshes/x.stl` 存在，
#   而文件在 `<install>/<pkg>/share/<pkg>/meshes/x.stl` ⇒ R = `<install>/<pkg>/share`
#   = `os.path.dirname(get_package_share_directory('<pkg>'))`。
#
# 为什么这样写（每一条都对应一个"不许弄坏别的路径"的约束）：
#   · **惰性**：查包只在 OpaqueFunction 里做（launch 运行期），不选这个槽位就永不执行
#     ⇒ 没构建过 `robot11` 也不会挡住别的槽位，`--show-args` 同样不受影响（与文件顶部
#     `_PackageShareFile` 的"描述构建期陷阱"同一原则）；
#   · **只在该加的时候加**：`_GAZEBO_MODEL_PATH_PKGS` 只登记「`<visual>` 里真有 `package://`
#     mesh」的槽位（默认模型与 hzmirm 全是 box/cylinder ⇒ 0 个，见文档 §3.2）
#     ⇒ 默认模型 / 其它 world 的 `GAZEBO_MODEL_PATH`、日志、时序**一个字节都不动**；
#   · **追加，不覆盖**：用 `AppendEnvironmentVariable`（`os.pathsep` 分隔），用户自己 export 的
#     值原样保留；用户若已经手工 export 过同一个目录（= 文档里那条 workaround），
#     这里认出来就不再重复追加（幂等）；
#   · **落在 gzserver/gzclient 上**：`AppendEnvironmentVariable` 改的是 launch 进程的 `os.environ`，
#     而 gazebo_ros 的 `gzserver.launch.py`/`gzclient.launch.py` 在**被执行时**读
#     `os.environ['GAZEBO_MODEL_PATH']` 并把它拼进 `ExecuteProcess` 的 `additional_env`
#     （`launch/descriptions/executable.py` 用 `copy.deepcopy(context.environment)` 建子进程环境）
#     ⇒ 只要本 Action **排在** include gzserver/gzclient 之前就一定生效（见 ld.add_action 的顺序）。
#   ⚠️ 顺带必须知道：`--show-args` 不执行 Action ⇒ 加了本段后 `--show-args` 的行为与以前逐字相同。
# =============================================================================

#: robot 槽位 → 该槽位需要在 `GAZEBO_MODEL_PATH` 上补解析根的 ROS 包名（只登记真有 mesh 的）。
#: 默认槽位与 hzmirm 都**不在表里** ⇒ 它们的 env 与改造前逐字节相同。
#: hzmi_rm_simulation（world / obstacle 的 `model://…`）**不在表里**：它自己的 package.xml
#: 早就 export 了 `${prefix}/meshes`（改造前就有，本次没动），本来就解析得到。
_GAZEBO_MODEL_PATH_PKGS = {
    'robot11': ('robot11',),        # 12 个 <visual> mesh + 2 个 package:// 引用（文档 §3.2）
}


def _pkg_model_path_exports(share_dir):
    """读出「这个包的**已安装** package.xml 里声明了哪些 gazebo_model_path」（只用于日志）。

    与 gazebo_ros 的 `gazebo_ros_paths.py` 走同一套代码/同一份文件（`catkin_pkg.parse_package`
    + `export.tagname == 'gazebo_ros'` + `${prefix}` 替换），所以日志里说的就是 gzserver/gzclient
    真正会拿到的东西。解析不了就返回 `[]`（= 不声称有）。
    """
    try:
        from catkin_pkg.package import parse_package
        package = parse_package(os.path.join(share_dir, 'package.xml'))
    except Exception:                                             # noqa: BLE001
        return []
    out = []
    for export in package.exports:
        if export.tagname == 'gazebo_ros' and 'gazebo_model_path' in export.attributes:
            out.append(export.attributes['gazebo_model_path'].replace('${prefix}', share_dir))
    return out


def _gazebo_model_path_setup(context):
    """按 `robot` 槽位把 `model://` 的本地解析根**追加**进 `GAZEBO_MODEL_PATH`（纯增量、惰性）。

    返回 launch Action 列表（OpaqueFunction 的约定）；不需要时返回 `[]`（= 什么都不做）。
    失败一律"降级为警告"，绝不 raise —— 解析根缺失只会让 mesh 加载退回旧行为（卡 + 没视觉），
    不该让整条 launch 起不来。

    ⚠️ 为什么**不**因为"包自己 export 了"就跳过追加（两条腿都要）：本函数跑在 gzserver/gzclient
       被 include **之前**，此刻 `os.environ` 里还**没有** gazebo_ros 扫出来的那些 export 项
       （那是 gzserver.launch.py 执行时才现算的）⇒ "包 export 在不在"这件事此刻无法从 env 判定，
       只能自己解析那份 package.xml；一旦那份 manifest 是旧的/解析不了，跳过就等于没修。
       所以默认**无条件追加**（只对"用户自己已经 export 过同一个目录"做幂等），
       包 export 那条只作为**其它 gazebo 入口**（裸 gzserver/gzclient、gz sim）的兜底。
    """
    slot = context.perform_substitution(LaunchConfiguration('robot')).strip()
    pkgs = _GAZEBO_MODEL_PATH_PKGS.get(slot, ())
    if not pkgs:
        return []                       # 其它槽位：env / 日志 / 时序都不动（逐字节不变）

    cur = os.environ.get('GAZEBO_MODEL_PATH', '')
    cur_items = [p for p in cur.split(os.pathsep) if p]
    added, covered, notes, broken = [], [], [], []
    for pkg in pkgs:
        try:
            share = get_package_share_directory(pkg)
        except PackageNotFoundError:
            broken.append('%s（当前 AMENT_PREFIX_PATH 里没有这个包：没构建过，'
                          '或构建后没重新 source install/setup.bash）' % pkg)
            continue
        share = os.path.abspath(share)
        root = os.path.dirname(share)
        exports = _pkg_model_path_exports(share)
        if exports:
            notes.append('%s 自己的 package.xml 也 export 了 %s（裸 gzserver/gzclient 入口靠它）'
                         % (pkg, '、'.join(exports)))
        # 幂等：用户手工 export 过同一个目录（文档里那条 workaround 的各种写法）、
        # 或上一次同槽位 launch 追加过 ⇒ 不重复追加。
        if any(os.path.realpath(p) == os.path.realpath(root) for p in cur_items):
            covered.append('%s（已在 GAZEBO_MODEL_PATH 里）' % root)
        elif not os.path.isdir(os.path.join(root, pkg)):
            broken.append('%s（%s/ 不存在 ⇒ install/ 不完整，请 colcon build 后重试）'
                          % (root, pkg))
        else:
            added.append(root)

    actions = []
    for why in broken:
        actions.append(LogInfo(msg='[gzmodel] ⚠️ 无法为 robot:=%s 补 model:// 解析根：%s；'
                                   '本次会退回旧行为（gzclient 可能卡在"等在线模型库"、'
                                   '车在 Gazebo 里没有 mesh）。' % (slot, why)))
    if added:
        actions.append(LogInfo(msg='[gzmodel] robot:=%s 需要 %s 的本地解析根：GAZEBO_MODEL_PATH '
                                   '追加 %s（追加，不覆盖原值：%s；有此根后 '
                                   'model://%s/meshes/… 不再回落在线模型库、不再 No mesh specified%s）'
                                   % (slot, '/'.join(pkgs), os.pathsep.join(added),
                                      (cur or '<空>'), pkgs[0],
                                      ('；' + '；'.join(notes)) if notes else '')))
        for root in added:
            actions.append(AppendEnvironmentVariable('GAZEBO_MODEL_PATH', root))
    if covered:
        actions.append(LogInfo(msg='[gzmodel] robot:=%s 的 model:// 解析根已存在（无需追加）：%s%s'
                                   % (slot, '；'.join(covered),
                                      ('；' + '；'.join(notes)) if notes else '')))
    return actions


# =============================================================================
# 「在上次基础上继续建图」= 存档（posegraph / PCD）+ 场地隔离守卫
# -----------------------------------------------------------------------------
# 用户需求原话：「能不能就是我在上次基础上继续建，手动指定一个地图名字去覆盖之类的」
#              「以后换地图换场地会不会有干扰」
#
# ① 续建机制（slam_toolbox 原生能力，我们只是把它接上）：
#      · 存档 = `<map_dir>/<name>.posegraph` + `.data`（`/slam_toolbox/serialize_map` 写出，
#        见 slam_toolbox/include/slam_toolbox/serialization.hpp:44-45）；
#      · 续建 = 给 slam_toolbox 传 `map_file_name=<base>`：节点起来时
#        `loadPoseGraphByParams()`（slam_toolbox_common.cpp:313）会反序列化同一对文件并**接着建**
#        （`map_file_name` 为空 ⇒ 什么都不加载 = 从零建，源码 349 行 `if (!filename.empty())`）；
#      · `map_start_pose` 只在"要反序列化"时有意义：它告诉 slam_toolbox
#        「机器人现在在**这张旧图**的哪个位姿」（START_AT_GIVEN_POSE ⇒ PROCESS_NEAR_REGION）。
#        本工程约定 `map` 系 = 出生点相对系 ⇒ 机器人每次都在出生点重生 ⇒ `[0,0,0]` 对任何 world 都对。
#
# ② 场地隔离（用户的第二个问题，也是最容易"静默毁图"的地方）：
#      存档旁边必须有一份 sidecar `<name>.meta.yaml`（记录 world / spawn / resolution / 写它的版本）。
#      续建前由 `map_asset_guard.py` 核对：world 不同、spawn 不同、或 sidecar 缺失（旧版存档）
#      ⇒ **默认拒绝并终止 launch**，除非显式 `map_allow_world_mismatch:=True`。
#
# ③ 判定放在哪、为什么放这里（"最简可靠"的取舍）：
#      放在 **launch 期**（OpaqueFunction，t=0 执行），而不是"另起一个守卫节点"：
#        · 守卫节点只能"打印/退出"，slam_toolbox 照样会把错的图 load 进来 ⇒ 拦不住；
#        · launch 期判定可以在**构造节点参数之前**决定"传不传 map_file_name"，并且能直接
#          `raise` ⇒ launch 收尾、退出码 1、错误信息完整（与文件顶部 `_PackageShareFile` 同一机制）；
#        · OpaqueFunction 是 Action，`ros2 launch ... --show-args` **不执行** Action
#          ⇒ 存档缺失/不匹配时 `--show-args` 依然正常（这一点已实测，见 docs/continue_mapping.md）。
#      副作用（有意的）：判定与"续建/从零"横幅在 t=0 就打印，而不是等 4 s 后 slam_toolbox 起来；
#      节点本身仍在 4 s 起（TimerAction 由 OpaqueFunction 内部返回，时序与原来逐字节一致）。
# =============================================================================

# 惰性加载守卫模块（模块本体在 share/rm_nav_bringup/scripts/map_asset_guard.py，与 shell 脚本共用同一份判定）
_MAP_GUARD_CACHE = {}


def _find_source_pkg_dir(share_dir):
    """从 `share/<pkg>/<sub>/` 里挑一个指向别处的软链，反推**仓库源码里的包目录**。

    为什么需要：`colcon build --symlink-install` 下 share 里的每个文件是**单独**的软链
    （目录本身是真的）⇒ 运行期新建的 `map/<name>.posegraph` 不会自动出现在 share 里。
    守卫模块因此优先从源码树加载（与 map_asset_guard.canonical_asset_dir() 同一套判断，
    互为兜底：即使忘了重编 rm_nav_bringup，launch 也能找到源码里的守卫）。
    """
    for sub in ('map', 'PCD', 'launch'):
        d = os.path.join(share_dir, sub)
        try:
            for entry in sorted(os.listdir(d)):
                p = os.path.join(d, entry)
                if os.path.islink(p):
                    real = os.path.realpath(p)
                    # …/src/<pkg>/<sub>/<file> → 上溯两级 = …/src/<pkg>
                    cand = os.path.dirname(os.path.dirname(real))
                    if os.path.isfile(os.path.join(cand, 'package.xml')):
                        return cand
        except OSError:
            continue
    return None


def _load_map_asset_guard(share_dir):
    if 'mod' in _MAP_GUARD_CACHE:
        return _MAP_GUARD_CACHE['mod']
    cands = [os.path.join(share_dir, 'scripts', 'map_asset_guard.py')]
    src_pkg = _find_source_pkg_dir(share_dir)
    if src_pkg:
        cands.append(os.path.join(src_pkg, 'scripts', 'map_asset_guard.py'))
    path = next((p for p in cands if os.path.isfile(p)), None)
    if path is None:
        raise RuntimeError(
            "[launch] 找不到地图存档守卫模块 map_asset_guard.py（试过：%s）\n"
            '  它是 rm_nav_bringup 包的一部分，安装方式：\n'
            '      colcon build --symlink-install --packages-select rm_nav_bringup\n'
            '      source install/setup.bash\n'
            '  （续建 + 场地隔离都靠它，缺了它宁可不起栈，也不要在没有隔离检查的情况下续建。）'
            % ' / '.join(cands))
    import importlib.util
    spec = importlib.util.spec_from_file_location('rm_map_asset_guard', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _MAP_GUARD_CACHE['mod'] = mod
    _MAP_GUARD_CACHE['path'] = path
    return mod


def _as_bool(text, what):
    t = str(text).strip().lower()
    if t in ('true', '1', 'yes', 'on'):
        return True
    if t in ('false', '0', 'no', 'off', ''):
        return False
    raise RuntimeError('[launch] 参数 %s 需要 True/False，收到 %r' % (what, text))


def _as_pose3(text, what):
    """把 `map_start_pose`（启动参数是字符串）解析成 [x, y, theta]。"""
    try:
        if isinstance(text, (list, tuple)):
            vals = [float(v) for v in text]
        else:
            import ast
            vals = [float(v) for v in ast.literal_eval(str(text).strip())]
    except Exception as e:
        raise RuntimeError('[launch] 参数 %s 解析失败（要形如 "[0.0, 0.0, 0.0]"）：%s' % (what, e))
    if len(vals) != 3:
        raise RuntimeError('[launch] 参数 %s 需要 3 个数 [x, y, theta]，收到 %r' % (what, text))
    return vals


def _as_float(text, what, default=None):
    """启动参数（字符串）→ float；空串 ⇒ default。"""
    t = str(text).strip()
    if t == '' and default is not None:
        return float(default)
    try:
        return float(t)
    except Exception as e:
        raise RuntimeError('[launch] 参数 %s 需要数字，收到 %r（%s）' % (what, text, e))


def _mapper_resolution(params_yaml, default=0.05):
    """读 slam_toolbox 参数文件里的 resolution（用于与存档 sidecar 比对，只报警不拦）。"""
    try:
        with open(params_yaml, 'r') as f:
            d = yaml.safe_load(f) or {}
        return float(d.get('slam_toolbox', {}).get('ros__parameters', {}).get('resolution', default))
    except Exception:
        return default


def _write_session_state(map_dir_path, payload):
    """把本次会话"存档落在哪、叫什么名字"写到 `<map_dir>/.session.yaml`。

    为什么需要这个文件：`map_archive.sh save` 在**另一个终端**里跑，它必须知道
    「这次 launch 到底用的是什么 map_name / world」。只靠 `ros2 param get` 是不够的：
    从零建图时 slam_toolbox 的 `map_file_name` 是空的（只有续建才有值），默认名
    （map_name 留空 ⇒ 取 world）在运行期完全不可见 ⇒ 会存到错名字上。
    写失败只警告（绝不因为一个记账文件挡住建图）。

    ⚠️ 2026-10-06 事故（存档被写到别人那套会话的名字上）的根因就是**这个文件是可变的**：
    任何后启动的 launch 都会覆盖它，而旧的 `save` 只做了一次 `kill -0 <pid>` 就照用。
    现在的分工是：
      · 本文件 = **次级**证据（还要配"活 launch 进程命令行 / slam_toolbox 的 map_file_name /
        记录的 pid+cmdline+起始时刻"里的至少一条证明，见 map_asset_guard.py::resolve_session）；
      · **活会话的第一手证据 = ROS 图上的播报器**（下面的 `map_session_announcer.py`，
        latched 话题 /map_session/info + 服务 /map_session/query）——
        它由本次 launch 起、随栈一起死，谁也覆盖不了。
    因此这里额外记 `session_id`（与播报器同一份）与 `launch_cmdline`（与活进程 argv 逐项可比）。
    """
    try:
        os.makedirs(map_dir_path, exist_ok=True)
        with open(os.path.join(map_dir_path, '.session.yaml'), 'w') as f:
            yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
        return True
    except Exception as e:                                        # pragma: no cover
        print('[map_archive] ⚠️ 写会话状态失败（不影响建图，但 map_archive.sh save 需要 --name）：%s' % e)
        return False


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
    # ★ 2026-10-07：Gazebo GUI / 在线模型库（默认值 = 改造前行为，见各自 DeclareLaunchArgument）
    gui = LaunchConfiguration('gui')
    gazebo_offline = LaunchConfiguration('gazebo_offline')
    spin_speed = LaunchConfiguration('spin_speed')

    ################################ robot_description parameters start ###############################
    # 平台外参（base_link↔livox）随仿真平台包 hzmi_rm_simulation 存放（规则③）
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('hzmi_rm_simulation'), 'config', 'measurement_params_sim.yaml')))
    # ★ 2026-10-07：robot:=<模型> 槽位（**opt-in**）。
    #   robot 留空（默认）= 现在的默认模型 sentry_robot_sim.xacro：命令字符串与逐字节行为都不变；
    #   robot:=hzmirm     = 用户给的哨兵 URDF（urdf/sentry_robot_hzmirm_sim.xacro = 其运动学逐字
    #                       保留 + 我们补的 inertial/IMU/雷达/底盘，见该文件头与 docs/robot_models.md）。
    #   惰性：路径/命令都在 Substitution.perform() 里才求值 ⇒ 不选这个槽位零成本，
    #   选错值时的可操作报错也只在真正执行时抛（与 _PackageShareFile 同一套设计原则）。
    robot_description = Command([_RobotXacroCommand(
        os.path.join(get_package_share_directory('rm_nav_bringup'), 'urdf'),
        launch_params['base_link2livox_frame']['xyz'],
        launch_params['base_link2livox_frame']['rpy'])])
    ################################# robot_description parameters end ################################

    ########################## linefit_ground_segementation parameters start ##########################
    # 参数已回归 linefit_ground_segmentation_ros 包自身 config/（R1）。
    # ★ 2026-10-07：按 robot 槽位选文件（默认模型那份**一个字都没改**）：
    #   · 默认模型 → segmentation_sim.yaml（sensor_height 0.226 = 默认模型实测）
    #   · hzmirm   → segmentation_sim_hzmirm.yaml（sensor_height = 该模型雷达离地**实测值**，
    #                且 gravity_aligned_frame="base_link"）。该模型雷达比默认模型高 ~0.57 m，
    #                照抄 0.226 会让地面线整体偏 0.57 m ⇒ 地面分割近乎全错；
    #                判据/实测见 docs/robot_models.md §3/§5。
    #   · robot11  → segmentation_sim_robot11.yaml（Phase 3 新增）：sensor_height = 雷达离地
    #                 **实测 0.2595**（默认模型 0.226 / hzmirm 0.80）、
    #                 gravity_aligned_frame=""（点云在源头已重力对齐 ⇒ 该键不需要、也不能开）。
    #                 ⚠️ 若用 livox_raise_m≠0 跑 B 方案，那份 YAML 的 sensor_height 必须同步
    #                    （0.2595 + raise，实测值），本 launch 不替改（避免第二个真源）。
    segmentation_params = _RobotSlotFile(
        'linefit_ground_segmentation_ros',
        ('config', 'segmentation_sim.yaml'),
        error_hint='该文件属于本次新增的 robot 槽位：'
                   'colcon build --symlink-install --packages-select '
                   'linefit_ground_segmentation_ros 后重试；'
                   '或去掉 robot:=<模型> 回到默认模型',
        slot_map={'hzmirm': ('config', 'segmentation_sim_hzmirm.yaml'),
                  'robot11': ('config', 'segmentation_sim_robot11.yaml')})
    ########################## linefit_ground_segementation parameters end ############################

    ########################## 可通行性判据（坡度/台阶）parameters start #############################
    # ★ 2026-10-07：**单一真源**的阈值文件。它同时被：
    #   ① 两个地面分割槽位（ground:=linefit / patchwork）当 ROS 参数文件读；
    #   ② 离线出先验图的 tools/scripts/mapping/pcd_to_nav2_map.py 读（--criteria-file）。
    #   ⇒ 实时代价图与先验图用同一套"局部地面高差 + 可行驶坡度"判据（缺陷 ③ 的修法）。
    #   放在 rm_nav_bringup 自己的 share 下（不依赖任何新包的构建），所以任何槽位组合都能启动。
    traversability_params = os.path.join(rm_nav_bringup_dir, 'config', 'traversability_criteria.yaml')
    # 容错：若 install/ 里还没有这份文件（只改了源码没重建 rm_nav_bringup），**不要**让整条 launch 挂掉
    # —— 节点的 C++ 兜底默认值与文件逐键相同（由 tools/scripts/regress/check_traversability_criteria.py
    # 断言），但会少一层"单一真源"。所以这里打印一条可操作的告警并继续。
    if os.path.isfile(traversability_params):
        traversability_params_list = [traversability_params]
    else:
        print('[bringup_sim] ⚠ 找不到 %s ⇒ 地面分割节点用内置默认判据（同值）。'
              '请 `colcon build --symlink-install --packages-select rm_nav_bringup` 后重试。'
              % traversability_params)
        traversability_params_list = []
    ########################## 可通行性判据（坡度/台阶）parameters end #################################

    ########################## 自击掩膜（self_mask）parameters start ##################################
    # ★ 2026-10-07 Phase 4：**按 robot 槽位**选"自击掩膜"参数文件（同一套 _RobotSlotFile 机制）。
    #   为什么需要：`robot:=robot11` 的雷达装在底盘凹槽里、360° 视场里最近的部件是云台
    #   l10/l11（表面离雷达 0.116~0.139 m）⇒ Gazebo 实测 **28.6% 的点 r<0.12 m**（§11.2）。
    #   这些近场自击点落在前瞻走廊的粗格 d=0 里 ⇒ 台阶残差被抬到 **0.164 m** ⇒ 限速器判成台阶
    #   ⇒ **车停着也被压到速度表地板 0.60 m/s**（Phase 4 基线实测，§12）。
    #   修法：判据层在建格**之前**把"落在机器人自己 collision 几何里、且离地 ≥3 cm"的点剔掉
    #   （几何定义/证据/离线验证见 tools/scripts/regress/robot11_self_mask.py 与 §12）。
    #   ⚠️ 只影响判据/限速：点云与 /segmentation/* 的**标签一个都不变**（自击是物理真实回波）。
    #   默认文件 = `traversability_self_mask.yaml`（self_mask_enable: false）⇒ 默认模型/其它槽位
    #   逐字节行为不变；只有 robot11 读 `traversability_self_mask_robot11.yaml`（开 + 113 个盒）。
    self_mask_params = _RobotSlotFile(
        'rm_nav_bringup',
        ('config', 'traversability_self_mask.yaml'),
        error_hint='该文件属于 Phase 4 的自击掩膜：'
                   '`colcon build --symlink-install --packages-select rm_nav_bringup` 后重试；'
                   '或删掉 launch 里这一路参数（回到"没有掩膜"的行为）',
        slot_map={'robot11': ('config', 'traversability_self_mask_robot11.yaml')})
    ########################## 自击掩膜（self_mask）parameters end ####################################

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
    # ★ 2026-10-07：nav2 的 2D 先验图**可显式指定**（`map_yaml:=/abs/path.yaml`）。
    #   动机：重跑建图之后要先在**新先验图**上验一遍（缺陷 ③ 的验收要求"旧图 vs 重跑图"对照），
    #   而 nav2 的默认路径是 map/<world>.yaml —— 没有这个参数就只能去覆盖那一份用户资产，
    #   既危险又不可对照。默认值与原来**逐字节相同**（map/<world>.yaml）⇒ 不传时行为不变。
    #   ⚠ 只有 2D 先验图（map_server 的 yaml）走这里；3D 点云/PCD 不经过 launch。
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([
            rm_nav_bringup_dir, 'map',
            PythonExpression(["'", LaunchConfiguration('world'), "' + '.yaml'"])]),
        description='nav2 用的 2D 先验图 yaml（绝对路径）。留空/不给 = <rm_nav_bringup share>/'
                    'map/<world>.yaml（= 现有行为）。用途：重跑建图后先用新图验一遍，'
                    '对照跑法 `map_yaml:=/abs/new.yaml`，不用覆盖 map/<world>.yaml')
    nav2_map_dir = LaunchConfiguration('map_yaml')
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
    # ★ 2026-10-07 Phase 3：公共段按槽位走一遍**深合并**（只有 robot11 会真的合并出临时文件；
    #   其它槽位原样返回同一份 base ⇒ 默认模型逐字节不变）。覆盖层只含代价图几何键
    #   （robot11 足印内切 0.300 / 外接 0.3565）—— 见 rm_nav_bringup/config/
    #   nav2_params_sim_robot11_costmap.yaml 的文件头与 docs/robot_models.md §11.6/§11.9。
    nav2_base_params_for_slot = _Nav2ParamsForSlot(
        os.path.join(nav2_params_dir, 'nav2_params_sim_base.yaml'),
        os.path.join(rm_nav_bringup_dir, 'config', 'nav2_params_sim_robot11_costmap.yaml'))
    nav2_params_launch_args = {
        'params_file': nav2_base_params_for_slot,           # 公共段（base；robot11 时 = 合并后）
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

    # ★ 2026-10-07 新增：Gazebo GUI 开关（**纯增量**：默认 True = 改造前行为）。
    #   为什么需要：用户 2026-10-07 反馈"gazebo 加载不出来"，而 RViz 一切正常。实测到两条
    #   独立原因（证据与复现见 docs/gazebo_gui_troubleshooting.md）：
    #     ① **`robot:=robot11` 的 `<visual>` mesh 让 gzclient 卡住**（不是"网络慢"这么笼统）：
    #        URDF 的 `package://robot11/...` 被 sdformat 改写成 `model://robot11/...`，
    #        该 URI 在 `GAZEBO_MODEL_PATH` 上解析不到 ⇒ `SystemPaths::FindFileURI()`
    #        无条件回落到 `ModelDatabase::GetModelPath(uri, forceDownload=true)` ⇒
    #        `GetModels()` 抢不到后台抓取线程的锁，于是打印
    #        "Waiting for model database update to complete..." 并**同步阻塞**。
    #        实测 stall：用户 19:50 = **48.03 s**、同机 20:13 = **76.34 s**、
    #        黑洞代理复现 = **99.67 s**（三次都是被 Ctrl-C / 拆代理打断的，**不设上限**）；
    #        同时 12 个 mesh 全部 `No mesh specified`（车在 Gazebo 里没有视觉）。
    #        ⇒ 治本是给 `model://robot11/...` 一个本地解析根（见该文档 §5.1）。
    #        ★ 2026-10-08：**已自动化** —— 本文件里的 `_gazebo_model_path_setup`（OpaqueFunction，
    #        排在 include gzserver/gzclient 之前）会按 slot 自动追加解析根，`robot:=robot11`
    #        现在**不需要**在命令行前缀 `GAZEBO_MODEL_PATH=…`；`robot11` 包自己也 export 了
    #        `${prefix}/..`（裸 gzserver/gzclient 入口的兜底）。实测见该文档 §5.1。下面这个
    #        `gazebo_offline` 开关因此只作"万一还是解析不到时不再等"的兜底。
    #     ② 本仓多个 bench/ab 脚本收尾会 `pkill -9 -x gzclient`（**全机**范围，不看
    #        GAZEBO_MASTER_URI）⇒ 别人正在看的 GUI 被打成 `exit code -9`（用户 16:55/16:56 那两次）。
    #   ⇒ gui:=False 一条命令无头跑，既不用事后 kill GUI，也不会被别人误杀。
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='True（默认）= 起 Gazebo GUI（gzclient），与改造前逐字相同；'
                    'False = 只起 gzserver（无头）。RViz 由 nav_rviz/lio_rviz 单独控制，不受本开关影响'
    )

    #   在线模型库：默认行为不变（用 gazebo 自带的 http://models.gazebosim.org/）。
    #   True = 把 GAZEBO_MODEL_DATABASE_URI 指到 http://127.0.0.1:1/（连接立刻被拒）⇒
    #   后台清单抓取立刻失败、`UpdateModelCacheImpl()` 直接返回 ⇒ GUI 不再可能卡在"等模型库"。
    #   ⚠️ 它**不会**让车在 Gazebo 里出现（那是 ① 的 mesh 解析问题）。
    #   ⚠️ 不要用空串：gazebo 的 ModelDatabase::GetURI() 里 `result[result.size()-1]`
    #   对空串是越界读（UB）；实测虽没崩，但 URI 会退化成 `//database.config`。
    declare_gazebo_offline_cmd = DeclareLaunchArgument(
        'gazebo_offline',
        default_value='False',
        description='False（默认，行为不变）= gazebo 自带的在线模型库地址；'
                    'True = 指向 http://127.0.0.1:1/ ⇒ 启动不再等在线模型库'
                    '（推荐给"窗口在但世界/模型不出来"；想彻底好还要给 model:// 一个本地解析根）'
    )

    declare_spin_speed_cmd = DeclareLaunchArgument(
        'spin_speed',
        # ★ 2026-10-07 Phase 3：默认值**按槽位**给（`robot:=robot11` ⇒ 0.0）。
        #   为什么：robot11 的四个轮子是**真 cylinder 碰撞**、云台 j10/j11 我们没有加控制器
        #   ⇒ "小陀螺"在仿真里既没有对应的执行机构、也不是这台车的实测行为；开着它只会让
        #   nav 看到的 base_link_fake 凭空转、而雷达（挂在 body 上）并不跟着转。
        #   本仓默认模型/其它槽位仍然 5.0（逐字节不变：只有 robot=='robot11' 才是 0.0）。
        default_value=_SlotSpinSpeedDefault(),
        description='fake_vel_transform 的小陀螺固定角速度 (rad/s)。'
                    '5.0 = 复现上游哨兵小陀螺行为（默认模型/其它槽位）；'
                    'robot:=robot11 的默认值 = **0.0**（该模型轮子是真 cylinder、云台无控制器，'
                    '仿真里既无执行机构也无实测依据）；0.0 = 角速度直通（等价普通 nav2）')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='RMUL2026',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    # ★ 2026-10-07：机器人模型槽位（**opt-in**，默认值 = 本仓现行模型 ⇒ 现有行为不变）。
    declare_robot_cmd = DeclareLaunchArgument(
        'robot',
        default_value='',
        description="机器人模型槽位（留空 = 本仓现行默认模型，行为与以前完全一致）："
                    "'' → urdf/sentry_robot_sim.xacro（雷达 base_link+0.12,0,0.175，离地 0.226）；"
                    "hzmirm → urdf/sentry_robot_hzmirm_sim.xacro（用户给的哨兵 URDF："
                    "雷达在云台头上 base_link+0,0,0.8、底盘 0.6x0.6x0.3、轮距 0.50/轴距 0.44；"
                    "运动学逐字保留，我们补 inertial/IMU/MID360/Livox 插件/底盘里程计）。"
                    "选 hzmirm 时 launch 会同时切到该模型的 linefit 参数文件并调整 lio_tf_adapter 杆臂。"
                    "robot11 → urdf/sentry_robot_robot11_sim.xacro（用户的哨兵 robot11：真 mesh 视觉、"
                    "真 inertial、雷达在底盘上斜 30° 下俯、离地 0.2595、足印内切 0.30/外接 0.3565；"
                    "碰撞件按实测换过 —— body 用 4 个 box 而不是 208 万面的原网格）。"
                    "选 robot11 时 launch 会调整 lio_tf_adapter 杆臂（含 30° 旋转补偿），"
                    "并切到该槽位专用的 linefit 参数（sensor_height=0.2595 + gravity_aligned_frame"
                    "=base_link）与 p2l 参数（target_frame=base_link，高度带改在重力系量）。"
                    "范围/数值对照见 docs/robot_models.md",
        choices=['', 'hzmirm', 'robot11'])

    # ★ 2026-10-07：robot:=robot11 的雷达 30° 安装轴 —— **上游两份材料自相矛盾，不猜，实测**：
    #   · URDF 原文 `body_to_livox` 的 rpy="0 -0.5236 0"（绕 y = pitch）
    #   · 同一台车的 SolidWorks CSV 写 Joint Origin Roll = -0.523598775598293（绕 x = roll）
    #   几何事实（docs/robot_models.md §9.5）：MID-360 方位 360° ⇒ 两个候选的**世界仰角谱与
    #   地面环半径完全相同**（都下俯 30°，盲区都是半径 ~0.34 m 的圆），只差一个绕 z 的旋转；
    #   **唯一判据是"最朝下的那一束指向 body 的哪个方位"**（roll → ±y / pitch → ±x）。
    declare_livox_tilt_axis_cmd = DeclareLaunchArgument(
        'livox_tilt_axis',
        default_value='roll',
        description='仅 robot:=robot11：雷达 30° 倾斜绕哪个轴（roll=绕 x / pitch=绕 y）。'
                    '默认 roll = SolidWorks CSV 的字面值（URDF 原文写的是 pitch，两者矛盾）。'
                    '⚠️ 对 360° 的 MID-360，两个取值在**仰角/地面可见性/盲区半径上完全等价**，'
                    '只改变"最朝下的方位"（roll→±y，pitch→±x）；实测见 docs/robot_models.md §10.5。'
                    '★ Phase 3：倾角实现位置 = `<gazebo><sensor><pose>`（射线真的斜），'
                    '而 livox_frame 这个帧是重力对齐的（理由：linefit 的 gravity_aligned_frame '
                    '路径在本仓有 C++ bug，见 docs/robot_models.md §11）',
        choices=['roll', 'pitch'])

    # ★ 2026-10-07 Phase 3：robot:=robot11 的 B 方案开关（把雷达整体抬高多少米）。
    #   默认 0.0 ⇒ 与上游 URDF 几何**逐字相同**（xacro 里 origin z = 0.15702816968305 + 0）。
    declare_livox_raise_m_cmd = DeclareLaunchArgument(
        'livox_raise_m',
        default_value=_LIVOX_RAISE_DEFAULT,
        description='仅 robot:=robot11：把 livox_frame（连同 imu_link）在 body 系里抬高的米数。'
                    '0.0 = 上游几何（A 方案：靠碰撞挖视锥让雷达看得见）；'
                    '非 0 = B 方案（保留 4 个 box，把雷达抬到顶板上方）—— 离线实测：'
                    '+0.06 m 才把自击从 96.6% 降到 19.8%、**+0.10 m 起才与 A 等价**。'
                    '⚠️ 抬高会改变雷达离地高度 ⇒ linefit 的 sensor_height 必须同步 = 0.2595 + 本值'
                    '（改 config/segmentation_sim_robot11.yaml；见 docs/robot_models.md §11 的复现步骤）')

    # ★ 2026-10-08：robot:=robot11 的**雷达安装方式**开关（opt-in；默认 = 2026-10-07 起的行为）。
    #   用户诉求原话："我就是要学斜放置怎么处理…雷达给我按我给你的放好，算法不管后面再看怎么调试，
    #   我要看到倾斜放置最原始的效果"。`plugin` 档把 30° 记在插件参数上（帧重力对齐 ⇒ 下游语义都对，
    #   但 TF/画的 mesh 是平的）；`urdf` 档把 30° 记回 URDF 关节（**实物的物理安装姿态，mesh 一起斜**）
    #   ⇒ 点云表达在斜的传感器系里、地面在点云里是 30° 斜面、`/scan` 高度带与 linefit 语义跟着变。
    #   ⚠️ 本开关**不动任何感知参数** —— 就是要看原始后果。取证与实测见 docs/tilted_lidar_fidelity.md。
    declare_robot11_mount_cmd = DeclareLaunchArgument(
        'robot11_mount',
        default_value='plugin',
        description='仅 robot:=robot11：30° 安装倾角**记在哪个坐标系上**。'
                    'plugin（默认，= 2026-10-07 起的行为）= `body_to_livox` 关节 rpy 0（帧重力对齐）'
                    '+ 插件 `<tilt_rpy>` 承担倾角（射线真的斜 30°，点云表达在水平的父 link 系；'
                    '代价：TF 与画出来的雷达 mesh 是平的）；'
                    'urdf = 倾角放回 URDF 关节（rpy = −0.5236 roll = 上游/CSV 字面值 = 实物的物理'
                    '安装姿态，**连 mesh 一起斜**）、插件 `<tilt_rpy>` = 单位阵 ⇒ 世界射线方向与 '
                    'plugin 档逐条相同，但点云表达在**斜的传感器系**里（地面是 30° 斜面）。'
                    '⚠️ urdf 档**不调任何感知参数**，就是要看倾斜放置最原始的效果。',
        choices=list(_LIVOX_MOUNTS))

    # ★ 2026-10-07 Phase 4：robot:=robot11 的**视觉 mesh 档位**。
    #   为什么需要（用户的 GUI 实测，见 docs/robot_models.md §12）：这台车的视觉是上游原始 STL，
    #   其中 `base_link.STL` 一项就是 **99 MiB / 2,078,226 三角形**（+ 云台 l11 8.8 MB / 17.6 万面）
    #   ⇒ 带 GUI 跑时 `gzclient` 被 SIGKILL（exit code -9）、`rviz2` 黑屏。
    #   decimated = 本相位的**专用视觉 LOD** `robot11_description/meshes/decimated/<link>.stl`
    #   （QEM 二次误差边折叠；整车 2,821,320 → **163,995** 面、141.07 → 8.20 MB；逐 mesh 的
    #   包围盒差/单向表面误差/三视剪影 IoU 全部量化在 inventory/visual_decimation.json）；
    #   full = 上游原始 STL（与 Phase 1~3 逐字节相同）。
    #   ⚠️ 只换 `<visual>`：`<collision>` 本来就是 76 个 box + 云台细盒 + cylinder（Phase 1/3 实测），
    #   一个字节都不动 ⇒ **物理/感知/契约完全不变**（`check_robot11_visual_slot.py` 的 A1/G5 断言）。
    declare_robot11_visual_cmd = DeclareLaunchArgument(
        'robot11_visual',
        default_value='decimated',
        description='仅 robot:=robot11：视觉 mesh 用哪一档。'
                    'decimated（默认）= **专用视觉 LOD** meshes/decimated/*.stl'
                    '（整车 2,821,320 → 163,995 面、141.07 → 8.20 MB，每 link ≤50k 面）；'
                    'full = 上游原始 STL（base_link.STL 103.91 MB / 207.8 万面）—— '
                    '用户 2026-10-07 的 GUI 跑就是这一档被杀掉 gzclient 的（§12/§13 有实测对照）',
        choices=['decimated', 'full'])

    # 云台角（度）：只对 robot:=hzmirm 生效，默认 0 = 与上游 URDF **数值等价**。    # 用途：试"雷达是不是斜放的/云台会不会动"这个假设（上游文件里两个云台关节都是 fixed、
    # head_to_lidar 的 rpy 是 0，即**没有**斜装）。pitch≠0 会破坏"雷达重力对齐"，
    # 这时 linefit 必须用 gravity_aligned_frame（该槽位的 YAML 已经配好 base_link）。
    declare_turret_yaw_deg_cmd = DeclareLaunchArgument(
        'turret_yaw_deg',
        default_value='0',
        description='仅 robot:=hzmirm：base_to_turret 的静态偏航角（度）。绕 z 转 ⇒ '
                    '雷达 z 轴仍朝上，**不影响** linefit/p2l 的重力对齐假设（实测见 docs/robot_models.md §3.3）')
    declare_turret_pitch_deg_cmd = DeclareLaunchArgument(
        'turret_pitch_deg',
        default_value='0',
        description='仅 robot:=hzmirm：turret_to_head 的静态俯仰角（度）。会**破坏**雷达重力对齐 ⇒ '
                    'linefit 必须配 gravity_aligned_frame=base_link（本槽位 YAML 已配）+ p2l 的高度带'
                    '在传感器系里被"拧斜"（实测劣化见 docs/robot_models.md §3.3）')

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

    # ★ 2026-10-06：新增 **地面分割槽位** `ground`。
    #   为什么需要它：现有链路的"什么算障碍"由 linefit 一家决定（每扇区直线拟合 + 硬
    #   `max_dist_to_line`）。linefit 在**斜面**上会把坡道/路缘判错（坡面点超出垂直容差 ⇒ 判成障碍
    #   ⇒ p2l 投出一圈"假墙" ⇒ 2D 图歪）。Patchwork++ 用**自适应平面拟合**（RNR/RVPF/GLE/TGR），
    #   对坡面天然友好 ⇒ 需要能在**不改任何别的槽位/参数文件**的前提下 A/B 两者。
    #   契约（两者必须一致，否则下游要跟着改）：同一个输入话题、同一对输出话题
    #   （/segmentation/obstacle + /segmentation/ground）、同一消息类型、同一帧处理、同一 QoS，
    #   且**每个输入点必属 ground 或 obstacle 之一**。
    #   ⇒ 由**条件**保证互斥：同一时刻只有一个分割器在跑，绝不会有第二个 /segmentation/* 发布者。
    #   默认 `linefit` = 现有行为**逐字节不变**（linefit 节点照旧无条件逻辑，只是多了一层恒真的条件）。
    #   对照表 / 参数来源 / A/B 实测 / 回退见 docs/ground_segmentation_slots.md。
    declare_ground_cmd = DeclareLaunchArgument(
        'ground',
        default_value='linefit',
        choices=['linefit', 'patchwork'],
        description='地面分割器槽位（A/B 对照用，只换"谁发 /segmentation/*"，其它一律不动）: '
                    'linefit = 已验证默认（linefit_ground_segmentation_ros，每扇区直线拟合 + '
                    'max_dist_to_line 硬容差；参数在 linefit_ground_segmentation_ros/config/'
                    'segmentation_sim.yaml）| '
                    'patchwork = Patchwork++（url-kaist/patchwork-plusplus，BSD-2-Clause，vendored '
                    '在 patchwork_ground_segmentation 包内，pin 3e6903a1 = v1.4.1；自适应平面拟合，'
                    '坡面更稳；参数在 patchwork_ground_segmentation/config/'
                    'ground_segmentation_sim.yaml）。'
                    '两者同契约 ⇒ 下游（pointcloud_to_laserscan / STVL / costmap）零改动、互斥切换；'
                    '回退 = 省略本参数或 ground:=linefit')

    # ★ 2026-10-06：以下 5 个参数 = 「在上次基础上继续建图」+ 场地隔离（详见文件中部那段说明与
    #   docs/continue_mapping.md）。全部是**新增**的、有默认值 ⇒ 不传时现有行为逐字节不变
    #   （唯一例外见 map_autocontinue：它默认 True，会在**同名存档已存在**时改成"续建"；
    #    本仓库现有 map/*.posegraph 只有 RMUC / RMUL 两份旧图，且它们没有 sidecar
    #    ⇒ 默认 world:=RMUL2026 时 map/RMUL2026.posegraph 根本不存在 ⇒ 仍然是从零建图）。
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='存档基名（续建/保存都用它，同名覆盖）。留空 = 取 <world>。'
                    '存档落在 map/<map_name>.{posegraph,data} + map/<map_name>.meta.yaml；'
                    '3D 先验落在 PCD/<map_name>.pcd + PCD/<map_name>.meta.yaml。'
                    '换场地怕串味时给每个 world 一个独立名字（如 RMUC2026_home / RMUL2026_home）')

    declare_map_autocontinue_cmd = DeclareLaunchArgument(
        'map_autocontinue',
        default_value='True',
        description='True = 若 map/<map_name>.posegraph 存在则**反序列化并接着建**（slam_toolbox '
                    'map_file_name + map_start_pose），不存在则从零建（两种情况都会在日志里明确写出路径）；'
                    'False = 永远从零建（旧存档原封不动，保存时才会被同名覆盖）')

    declare_map_start_pose_cmd = DeclareLaunchArgument(
        'map_start_pose',
        default_value='[0.0, 0.0, 0.0]',
        description='续建时告诉 slam_toolbox「机器人现在在**那张旧图**的哪个位姿」（x, y, theta，map 系）。'
                    '本工程 map 系 = 出生点相对系 ⇒ 机器人在出生点重生时 [0,0,0] 对任何 world 都正确；'
                    '只有"这次不在出生点起"才需要改')

    declare_map_allow_world_mismatch_cmd = DeclareLaunchArgument(
        'map_allow_world_mismatch',
        default_value='False',
        description='False（默认）= 存档 sidecar 记录的 world/出生点与本次不一致、或存档没有 sidecar'
                    '（旧版存档）时**拒绝加载并终止 launch**（防止把两个场地的图叠在一起）；'
                    'True = 显式承担风险、跳过隔离检查（日志会一直提醒）。'
                    '只想从零建图请改用 map_autocontinue:=False，不要用这个参数')

    declare_cloud_accumulator_cmd = DeclareLaunchArgument(
        'cloud_accumulator',
        default_value='False',
        description='仅 mode:=mapping 生效。True = 额外起 3D 点云累加器（cloud_accumulator 包）：'
                    '把 LiDAR 点云按 TF 变换到 map 系、体素下采样后累积，服务 save/load 让 '
                    'PCD/<map_name>.pcd 这份 3D 先验也能**跨会话续建**（同样受 world/spawn 守卫保护）。'
                    '默认 False：它是只读消费者（不发 /map、不发 TF，不碰 map→odom 与 /segmentation 契约），'
                    '但属新增路径、还没做长跑验收 ⇒ 先关着，要用显式打开')

    # ★ 2026-10-06（事故当天追加）：续建后的**一次性**一致性检查（详见 docs/continue_mapping.md §8）
    #   动机：场地隔离守卫只管"world/spawn 对不对"，管不了"图本身歪了"——
    #   当天用户续建的 map/RMUC2026.posegraph 里已经含有一段 LIO 退化（ATE max 1.15 m），
    #   加载进来的图与真实场地不一致 ⇒ 建出来的图/定位"飘"。
    #   做法：续建时额外起一个**一次性**小节点（rm_nav_bringup/scripts/map_resume_check.py），
    #   它只订阅 /tf、不发任何话题/TF（「map→odom 单一发布者」契约逐字不变）；等 TF 可用 + 静置/
    #   收敛窗口（map_resume_check_delay）结束后判两条：① 自基线以来 map→odom 被修正了多少
    #   （图歪了会把位姿拉走，这条不需要机器人静止）② 实测 map→base_link 与**存档记录的
    #   map_start_pose** 的绝对偏差（只在"一步没动"时参与，抓"起点被挪/存档被 doctored"）。
    #   超阈值就喊响并可（strict）直接收栈 —— 两条判据的取舍见 docs/continue_mapping.md §8.3。
    #   ⚠️ 它由 launch 在 t=0 与其它节点**并行**起，自己的定时器负责等待 ⇒ 不推迟任何节点的启动；
    #      只有真的续建（map_file_name 有值）时才起；从零建图时压根不创建这个节点。
    declare_map_resume_check_cmd = DeclareLaunchArgument(
        'map_resume_check',
        default_value='True',
        description='True（默认）= 续建后做一次"加载的位姿图 vs 出生点"一致性检查（只喊不拦）；'
                    'False = 完全关掉这个检查（只续建、不体检）')

    declare_map_resume_check_delay_cmd = DeclareLaunchArgument(
        'map_resume_check_delay',
        default_value='8.0',
        description='续建一致性检查的**静置/收敛窗口**（秒）：从"第一次拿到 map→base_link"（= slam_toolbox '
                    '已反序列化完并开始发 TF）起算，等这么久再采样，让扫描匹配先收敛。')
    declare_map_resume_check_strict_cmd = DeclareLaunchArgument(
        'map_resume_check_strict',
        default_value='False',
        description='True = 检查不通过时**直接收栈**（检查节点退出码 1 ⇒ launch 收掉所有节点）；'
                    'False（默认）= 只打响亮 WARNING，继续跑')
    declare_map_resume_check_pos_tol_cmd = DeclareLaunchArgument(
        'map_resume_check_pos_tol',
        default_value='0.5',
        description='位置偏差阈值（m）。实测健康的 LIO 重启一致性在厘米级，0.5 m 留给扫描匹配收敛与'
                    '里程计补偿的余量；当天事故的 ATE max 是 1.15 m ⇒ 0.5 能抓住。')
    declare_map_resume_check_yaw_tol_deg_cmd = DeclareLaunchArgument(
        'map_resume_check_yaw_tol_deg',
        default_value='10.0',
        description='偏航偏差阈值（度），同上口径。')

    # ★ 2026-10-06（事故后追加）：**会话播报器**（map_session_announcer.py）。
    #   动机：`map_archive.sh save`（另一个终端）以前靠读 `<map_dir>/.session.yaml` 猜名字，
    #   而那个文件会被任何后启动的 launch 覆盖 —— 当天用户给自己那套栈（map_name:=RMUC2026_v2）
    #   跑 save，却被写到了并发测试栈的名字（RMUC2026_dropab_ab_g_long）上（31.89 MB 位姿图）。
    #   播报器把"本次会话是谁"挂到 ROS 图上（latched 话题 /map_session/info + 服务
    #   /map_session/query），并自检"图上是否只有一个 /slam_toolbox、且它提供
    #   /slam_toolbox/serialize_map"⇒ save 拿到的名字**来自活栈本身**，谁也没法覆盖。
    #   它是**只发布**的附加节点：不发 /map、不发 TF、不订阅、不改任何既有节点/时序；
    #   关掉它也只是少一条证据（save 会退到"活 launch 进程命令行 / .session.yaml + 活性证明"，
    #   两条都不成立就拒绝写盘）。详见 docs/continue_mapping.md §11。
    declare_map_session_announce_cmd = DeclareLaunchArgument(
        'map_session_announce',
        default_value='True',
        description='True（默认）= 起会话播报器 map_session（只广播"本次 launch 用了哪个 map_name/'
                    'world/存档基名/出生点/session_id"，供 map_archive.sh save 当场核对；'
                    '只发布，不影响建图）；False = 不起（save 需要 --name 或依赖活进程/文件证据）')

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
            # ★ 2026-10-07：Gazebo GUI / 在线模型库开关（默认值 = 改造前行为）
            'gui': gui,
            'gazebo_offline': gazebo_offline,
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
        parameters=[segmentation_params, *traversability_params_list, self_mask_params,
                    {'use_sim_time': use_sim_time}],
        # ★ 2026-10-06：地面分割槽位 ground（默认 linefit ⇒ 本节点照旧启动，行为不变）。
        condition=LaunchConfigurationEquals('ground', 'linefit'),
    )

    # ★ 2026-10-06：ground:=patchwork —— 与上面 linefit 节点**同契约**的替代分割器。
    #   参数文件用惰性 substitution 解析（_PackageShareFile）：这样"没构建过
    #   patchwork_ground_segmentation 包"时，只要不选这个槽位，`--show-args` 与其它槽位组合
    #   都不会被它挡住（理由见本文件顶部那段"描述构建期陷阱"）。
    bringup_patchwork_ground_segmentation_node = Node(
        package='patchwork_ground_segmentation',
        executable='patchwork_ground_segmentation_node',
        name='ground_segmentation',      # 与 linefit 节点同名（两者互斥 ⇒ 不会重名冲突）
        output='screen',
        parameters=[
            _PackageShareFile('patchwork_ground_segmentation', 'config',
                              'ground_segmentation_sim.yaml'),
            *traversability_params_list,
            self_mask_params,
            {'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('ground', 'patchwork'),
    )

    # ★ 2026-10-07 Phase 3：p2l **不需要**槽位专用参数了（默认那份一个字节都没改）。
    #   为什么曾经想改：p2l 的 min_height/max_height 是在**点云自带帧**里量的高度带，
    #   而雷达斜 30° ⇒ 那条带子在重力系里是斜的（1 m 处约 ±0.5 m 偏差、随方位变号）。
    #   最终修法不在 p2l：Phase 3 把**点云本身**做成重力对齐的（livox_frame 的 rpy=0、
    #   倾角搬到 <sensor><pose>，见 sentry_robot_robot11_sim.xacro 与 docs/robot_models.md §11）
    #   ⇒ p2l 的高度带自动就是"离传感器的真高度"，且**保住了 target_frame="" 这个加固**
    #   （不做 TF、不建 MessageFilter —— 2026-09-23 刻意避开的那类故障面）。
    #   实测：robot:=robot11 的 /scan frame_id = livox_frame、10 Hz、有 >4 m 的波束。
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
            LaunchConfiguration('lio'), "' != 'small_point_lio' and '",
            LaunchConfiguration('robot'), "' != 'hzmirm' and '",
            LaunchConfiguration('robot'), "' != 'robot11'"])),
        package='lio_tf_adapter',
        executable='lio_tf_adapter_node',
        name='lio_tf_adapter',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('lio_tf_adapter'), 'config', 'lio_tf_adapter.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # ★ 2026-10-07：robot:=hzmirm 的 lio_tf_adapter —— **同一个包/可执行文件/节点名**，
    #   唯一区别是杆臂补偿量（与上面那条 condition 互斥 ⇒ 永远只有一个在跑、同一时刻只有一个
    #   odom→base_link 发布者；写法照抄本文件里 ground:=linefit / patchwork 的互斥两节点模式）。
    #   为什么必须换：适配器的 xyz = T_body←base_link = -(imu_link 在 base_link 系下的位置)。
    #     默认模型：imu_link = base_link+(0.12, 0, 0.125) ⇒ [-0.12, 0, -0.125]
    #     hzmirm  ：imu_link = base_link+(0.00, 0, 0.750) ⇒ [ 0.00, 0, -0.750]
    #   不换的后果：base_link 在 odom 里被抬到 z≈0.675（水平直行时 x/y/yaw 不受影响，
    #   但俯仰/侧倾一动就按 0.675 m 的假杆臂放大成 x/y 误差，RViz 里车也悬空）。
    lio_tf_adapter_hzmirm_node = Node(
        condition = IfCondition(PythonExpression([
            "'", LaunchConfiguration('lio'), "' != 'none' and '",
            LaunchConfiguration('lio'), "' != 'cartographer' and '",
            LaunchConfiguration('lio'), "' != 'small_point_lio' and '",
            LaunchConfiguration('robot'), "' == 'hzmirm'"])),
        package='lio_tf_adapter',
        executable='lio_tf_adapter_node',
        name='lio_tf_adapter',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('lio_tf_adapter'), 'config', 'lio_tf_adapter.yaml'),
                    {'use_sim_time': use_sim_time,
                     'xyz': [0.0, 0.0, -0.75],   # hzmirm 的 imu_link 在 base_link+0,0,0.75
                     'rpy': [0.0, 0.0, 0.0]}]
    )

    # ★ 2026-10-07：robot:=robot11 的 lio_tf_adapter —— 同包/同可执行文件/同节点名，
    #   唯一区别是杆臂（xyz）**和姿态补偿（rpy）**。与前两条互斥 ⇒ 同一时刻只有一个
    #    odom→base_link 发布者（契约不变）。
    #   为什么这次连 rpy 都要给：robot11 的雷达是**斜 30°** 装的，而 imu_link 按
    #   【MID-360 的 IMU 与雷达同壳】这条物理事实跟着雷达一起斜
    #   （好处：三份 LIO 配置的 extrinsic_T=[0,0,0.05]/extrinsic_R=I **一个字节都不用改**）。
    #   ⇒ LIO 的 body 系 = 斜 30° 的 IMU 系 ⇒ 必须用 rpy 把它转回水平的 base_link。
    #   数值由 _LioAdapterRobot11 从 URDF 几何**算**出来（不是手抄），随 livox_tilt_axis 变；
    #   用 Substitution 惰性求值（照抄本文件 _PackageShareFile 的设计原则：不选这个槽位零成本）。
    lio_tf_adapter_robot11_node = Node(
        condition = IfCondition(PythonExpression([
            "'", LaunchConfiguration('lio'), "' != 'none' and '",
            LaunchConfiguration('lio'), "' != 'cartographer' and '",
            LaunchConfiguration('lio'), "' != 'small_point_lio' and '",
            LaunchConfiguration('robot'), "' == 'robot11'"])),
        package='lio_tf_adapter',
        executable='lio_tf_adapter_node',
        name='lio_tf_adapter',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('lio_tf_adapter'), 'config', 'lio_tf_adapter.yaml'),
                    {'use_sim_time': use_sim_time,
                     'xyz': [_LioAdapterRobot11('xyz')],   # T_imu_link←base_link（30° 倾斜下挂 0.05 m）
                     'rpy': [_LioAdapterRobot11('rpy')]}]  # 该 30° 旋转的逆 ⇒ 斜的 body 系转回水平
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

    # ===== 在线 2D 建图后端（slam_toolbox）+ 「续建 + 场地隔离」=====
    # 与改造前的差别只有两点：① 节点在 OpaqueFunction 里构造（这样才能按"存档在不在/合不合规"
    # 决定传不传 map_file_name）；② 判定/横幅提前到 t=0（节点本身仍由内部 TimerAction 延后 4 s）。
    # 节点的包/可执行文件/节点名/参数文件一律没变 ⇒ /map、map→odom 的"唯一发布者"契约不变。
    def _launch_slam_toolbox_mapping(context, *args, **kwargs):
        guard = _load_map_asset_guard(rm_nav_bringup_dir)
        world_v = context.perform_substitution(LaunchConfiguration('world')).strip()
        name_v = context.perform_substitution(LaunchConfiguration('map_name')).strip() or world_v
        autocontinue = _as_bool(context.perform_substitution(LaunchConfiguration('map_autocontinue')),
                                'map_autocontinue')
        allow_mismatch = _as_bool(
            context.perform_substitution(LaunchConfiguration('map_allow_world_mismatch')),
            'map_allow_world_mismatch')
        start_pose = _as_pose3(context.perform_substitution(LaunchConfiguration('map_start_pose')),
                               'map_start_pose')
        map_dir_path = guard.map_dir(rm_nav_bringup_dir)
        base = os.path.join(map_dir_path, name_v)
        resolution = _mapper_resolution(slam_toolbox_mapping_file_dir)

        verdict = None
        if autocontinue:
            verdict = guard.verify(base, name_v, world_v, kind=guard.KIND_POSEGRAPH,
                                   allow_mismatch=allow_mismatch,
                                   map_start_pose=start_pose, resolution=resolution)
            if verdict['code'] == 'fresh':
                banner = (verdict['message'] +
                          '\n  · 本次会话的存档名 = %s（world=%s）；保存用：'
                          ' tools/scripts/mapping/map_archive.sh save'
                          '\n  · 想续建别的存档就加 map_name:=<名字>；想永远从零建就加 map_autocontinue:=False'
                          % (name_v, world_v))
            elif verdict['may_load']:
                banner = verdict['message']
            else:
                # ★ 拒绝：直接抛 ⇒ launch 收尾、退出码 1、错误整段打到 [ERROR]（不静默、不半启动）
                raise RuntimeError(verdict['message'])
        else:
            banner = ('[map_archive] map_autocontinue:=False ⇒ 本次**从零建图**，不加载任何旧状态'
                      '（存档名仍为 %s，保存时会同名覆盖 map/%s.*）' % (name_v, name_v))

        params = {'use_sim_time': use_sim_time}
        if verdict is not None and verdict['may_load']:
            params['map_file_name'] = base          # ← slam_toolbox 反序列化并接着建（续建的全部秘密）
            params['map_start_pose'] = start_pose   # ← 机器人在旧图里的位姿（出生点相对系 ⇒ [0,0,0]）
        node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_toolbox_mapping_file_dir, params],
        )
        # ★ 2026-10-06（事故后）：本次会话的**唯一身份号**。`.session.yaml` 与 ROS 图上的
        #   会话播报器共用它 ⇒ `save` 能一眼看出"文件里记的"和"活栈上广播的"是不是同一次。
        session_id = uuid.uuid4().hex[:12]
        _write_session_state(map_dir_path, {
            'map_name': name_v,
            'world': world_v,
            'map_dir': map_dir_path,
            'pcd_dir': guard.pcd_dir(rm_nav_bringup_dir),
            'map_start_pose': start_pose,
            'map_autocontinue': autocontinue,
            'map_allow_world_mismatch': allow_mismatch,
            'resumed': bool(verdict is not None and verdict['may_load']),
            'archive_base': base,
            'session_id': session_id,
            'session_pid': os.getpid(),
            'launch_cmdline': list(sys.argv),     # save 用它把"记录的那次 launch"与活进程 argv 逐项对比
            'started_at': guard.now_iso(),
            'written_by': 'bringup_sim.launch.py',
        })
        actions = [LogInfo(msg=banner), TimerAction(period=4.0, actions=[node])]

        # ★ 2026-10-06（事故后）：**会话播报器**（只发布，不改变既有节点集/时序）。
        #   动机：`save` 在另一个终端跑，它必须知道"现在这套栈是谁"，而 `.session.yaml`
        #   会被任何后启动的 launch 覆盖（那天 31.89 MB 位姿图就是这么写到别人名字上的）。
        #   播报器随本次 launch 生、随本次 launch 死 ⇒ 身份不可被后人改写；它自己还会核对
        #   "图上是否只有一个 /slam_toolbox 且它提供 /slam_toolbox/serialize_map"。
        #   不需要它时：map_session_announce:=False（save 会退到"活 launch 进程命令行 /
        #   .session.yaml+活性证明"，两条都不成立就拒绝）。
        if _as_bool(context.perform_substitution(LaunchConfiguration('map_session_announce')),
                    'map_session_announce'):
            actions.append(Node(
                package='rm_nav_bringup',
                executable='map_session_announcer.py',
                name='map_session',
                output='screen',
                parameters=[{
                    'map_name': name_v,
                    'world': world_v,
                    'archive_base': base,
                    'map_start_pose': '[%r, %r, %r]' % tuple(start_pose),
                    'resumed': bool(verdict is not None and verdict['may_load']),
                    'autocontinue': autocontinue,
                    'allow_world_mismatch': allow_mismatch,
                    'started_at': guard.now_iso(),
                    'session_id': session_id,
                    'launch_pid': os.getpid(),
                }],
            ))

        # ★ 2026-10-06：**续建成功时**才追加"一次性一致性检查"节点（从零建图不创建它）。
        #   期望值取**存档 sidecar 里记录的 map_start_pose**（不是本次传的 map_start_pose）：
        #   要比的是"存档说它自己是什么样" vs "加载后现实是什么样"，用本次参数比等于自己跟自己比。
        if verdict is not None and verdict['may_load']:
            check_on = _as_bool(context.perform_substitution(LaunchConfiguration('map_resume_check')),
                                'map_resume_check')
            if check_on:
                man = verdict.get('manifest') or {}
                exp = [float(v) for v in (man.get('map_start_pose') or [0.0, 0.0, 0.0])]
                sp = man.get('spawn_pose') or {}
                check_node = Node(
                    package='rm_nav_bringup',
                    executable='map_resume_check.py',        # install(PROGRAMS …) 装到 lib/rm_nav_bringup/
                    name='map_resume_check',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
                        'expected_pose': exp,
                        'archive_name': name_v,
                        'archive_base': base,
                        'archive_world': man.get('world') or world_v,
                        'archive_spawn': [float(sp.get('x', 0.0)), float(sp.get('y', 0.0)),
                                          float(sp.get('z', 0.0)), float(sp.get('yaw', 0.0))],
                        'pcd_path': os.path.join(guard.pcd_dir(rm_nav_bringup_dir), name_v + '.pcd'),
                        'check_delay': _as_float(context.perform_substitution(
                            LaunchConfiguration('map_resume_check_delay')), 'map_resume_check_delay'),
                        'pos_tol': _as_float(context.perform_substitution(
                            LaunchConfiguration('map_resume_check_pos_tol')), 'map_resume_check_pos_tol'),
                        'yaw_tol_deg': _as_float(context.perform_substitution(
                            LaunchConfiguration('map_resume_check_yaw_tol_deg')),
                            'map_resume_check_yaw_tol_deg'),
                        'strict': _as_bool(context.perform_substitution(
                            LaunchConfiguration('map_resume_check_strict')), 'map_resume_check_strict'),
                    }],
                )
                actions.append(check_node)

                # strict 模式：检查节点以**非 0**退出码结束 ⇒ 收栈（非 0 才收，所以"检查通过后正常退出"
                # 不会误伤；退出码 0 时这个 handler 返回空列表，什么都不做）。
                def _on_check_exit(event, _context, _name=name_v):
                    rc = getattr(event, 'returncode', 0)
                    if rc not in (0, None):
                        return [LogInfo(msg=('[map_resume_check] ❌ 续建一致性检查不通过（退出码 %s）⇒ '
                                             'map_resume_check_strict:=True，收栈。存档 %s 未被写入。'
                                             % (rc, _name))),
                                Shutdown(reason='map_resume_check strict failure')]
                    return []

                actions.append(RegisterEventHandler(OnProcessExit(target_action=check_node,
                                                                  on_exit=_on_check_exit)))
        return actions

    start_mapping = GroupAction(
        condition=slam_mapping_condition,
        actions=[OpaqueFunction(function=_launch_slam_toolbox_mapping)],
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

    # ===== 3D 点云累加器（cloud_accumulator:=True，仅 mode:=mapping）=====
    # 目的：让 **3D 先验**（PCD/<map_name>.pcd）也能跨会话续建，同样受 world/spawn 守卫保护。
    # 契约：它只**订阅**点云 + 查 TF，不发 /map、不发任何 TF、不发 /segmentation
    #   ⇒ 「map→odom 单一发布者」「/segmentation/* 单一发布者」两条契约逐字不变。
    # 默认关闭（理由见 declare_cloud_accumulator_cmd 的 description）。
    cloud_accumulator_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'mapping' and '",
        LaunchConfiguration('cloud_accumulator'), "' == 'True'"]))

    def _launch_cloud_accumulator(context, *args, **kwargs):
        guard = _load_map_asset_guard(rm_nav_bringup_dir)
        world_v = context.perform_substitution(LaunchConfiguration('world')).strip()
        name_v = context.perform_substitution(LaunchConfiguration('map_name')).strip() or world_v
        autocontinue = _as_bool(context.perform_substitution(LaunchConfiguration('map_autocontinue')),
                                'map_autocontinue')
        allow_mismatch = _as_bool(
            context.perform_substitution(LaunchConfiguration('map_allow_world_mismatch')),
            'map_allow_world_mismatch')
        # 3D 先验一旦越过场地边界同样是"把两张图叠在一起"⇒ 走同一份 sidecar / 同一套判定
        # （节点内部还会在 load/save 前各查一次；这里先查是为了在**起栈时**就把问题喊出来，
        #   但 3D 存档不匹配**不终止 launch** —— 2D 位姿图才是主线，用户可能只想建 2D）。
        verdict = None
        if autocontinue:
            verdict = guard.verify(os.path.join(guard.pcd_dir(rm_nav_bringup_dir), name_v),
                                   name_v, world_v, kind=guard.KIND_PCD,
                                   allow_mismatch=allow_mismatch)
            if verdict['may_load']:
                banner = verdict['message']
            elif verdict['code'] == 'fresh':
                banner = ('[map_archive] 3D 先验：没有 PCD/%s.pcd ⇒ 本次从零点云开始累积'
                          '（cloud_accumulator 的 save/load 服务在 ~/save、~/load）' % name_v)
            else:
                # 3D 这一路被拒绝**不终止 launch**（2D 位姿图才是主线，用户可能只想建 2D）；
                # 节点内部还会在 load/save 前再各查一次 ⇒ 它不会加载、也不会覆盖那份 PCD。
                banner = ('[map_archive] ⚠️ 3D 先验这一路被守卫拒绝（**2D 建图不受影响**）：\n' +
                          verdict['message'])
        else:
            banner = ('[map_archive] 3D 先验：map_autocontinue:=False ⇒ 不加载 PCD/%s.pcd，'
                      '从零点云开始累积' % name_v)
        node = Node(
            package='cloud_accumulator',
            executable='cloud_accumulator_node',
            name='cloud_accumulator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_name': name_v,
                'world': world_v,
                'allow_world_mismatch': allow_mismatch,
                'autoload': autocontinue,
                'pcd_dir': guard.pcd_dir(rm_nav_bringup_dir),
                # 与 PCD/ 既有资产的体素口径一致（0.10 m，见 docs/mapping_small_point_lio.md §6.2）
                'voxel_size': 0.10,
            }],
        )
        return [LogInfo(msg=banner), TimerAction(period=6.0, actions=[node])]

    start_cloud_accumulator = GroupAction(
        condition=cloud_accumulator_condition,
        actions=[OpaqueFunction(function=_launch_cloud_accumulator)],
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
    # ★ 2026-10-07：Gazebo GUI / 在线模型库（默认值 = 改造前行为；纯增量，无节点集合/时序变化）
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_gazebo_offline_cmd)
    # ★ 2026-10-07 Phase 3：`robot` 必须**声明在 spin_speed 之前** —— spin_speed 的默认值是
    #   "按槽位给"的（robot11 ⇒ 0.0），而 launch 的声明动作是**按顺序执行**的：
    #   声明顺序反了就读不到 robot（实测会退化成旧默认 5.0，甚至抛 SubstitutionFailure）。
    ld.add_action(declare_robot_cmd)
    ld.add_action(declare_spin_speed_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_turret_yaw_deg_cmd)
    ld.add_action(declare_turret_pitch_deg_cmd)
    # ★ 2026-10-07：robot:=robot11 的雷达安装轴（roll | pitch；上游两份材料矛盾 ⇒ 不猜）
    ld.add_action(declare_livox_tilt_axis_cmd)
    ld.add_action(declare_livox_raise_m_cmd)
    # ★ 2026-10-08：robot:=robot11 的雷达**安装方式**（倾角记在插件 vs 记在 URDF 关节）
    ld.add_action(declare_robot11_mount_cmd)
    # ★ Phase 4：robot:=robot11 的视觉 mesh 档位（decimated | full）
    ld.add_action(declare_robot11_visual_cmd)
    # 选了哪个模型，日志里给一句收据（两条互斥；默认那条不改变任何行为）
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot'), "' == ''"])),
        msg='[robot] 模型槽位 = 默认（urdf/sentry_robot_sim.xacro）'
            '：雷达 base_link+0.12,0,0.175，linefit sensor_height=0.226（现行标定，未改）'))
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot'), "' == 'hzmirm'"])),
        msg=['[robot] 模型槽位 = hzmirm（urdf/sentry_robot_hzmirm_sim.xacro）：用户给的哨兵 URDF，'
             '雷达在云台头上、离地 ~0.80 m（比默认模型高 0.57 m）⇒ '
             '本次 linefit 用 segmentation_sim_hzmirm.yaml（sensor_height 重新标定 + '
             'gravity_aligned_frame=base_link），lio_tf_adapter 杆臂改为 [0,0,-0.75]。'
             '云台角 turret_yaw_deg=', LaunchConfiguration('turret_yaw_deg'),
             ' turret_pitch_deg=', LaunchConfiguration('turret_pitch_deg'),
             '（0 = 与上游 URDF 数值等价）。数值对照/回滚见 docs/robot_models.md']))
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot'), "' == 'robot11'"])),
        msg=['[robot] 模型槽位 = robot11（urdf/sentry_robot_robot11_sim.xacro）：用户的哨兵，'
             '真 mesh 视觉 + 真 inertial；雷达在**底盘**上（不在云台）斜 30° 下俯，离地 0.2595 m；'
             '足印内切 0.300 / 外接 0.3565 m；质量 9.5521 kg。',
             '  · 雷达倾斜轴 livox_tilt_axis=', LaunchConfiguration('livox_tilt_axis'),
             '（roll = SolidWorks CSV 的字面值；两条候选在仰角/盲区上等价，只差最朝下的方位）',
             '  · 视觉 mesh：robot11_visual=', LaunchConfiguration('robot11_visual'),
             '（decimated = **专用视觉 LOD** meshes/decimated/*.stl：整车 2,821,320 → 163,995 面、'
             '141.07 → 8.20 MB，逐 mesh 的包围盒差/表面误差/剪影 IoU 量化在 '
             'inventory/visual_decimation.json；full = 上游原始 STL。两者只差 `<visual>` 的 '
             'mesh 文件名，`<collision>`/`<inertial>` 逐字节相同 ⇒ 物理/感知/契约不变）',
             # ★ Phase 4 更正：上一轮这里写的是"linefit 参数**没有**跟着切" —— 那是**错的**
             #   （Phase 3 已经用 _RobotSlotFile 的 slot_map 切了），横幅在骗人。
             #   现在改为**运行时从真正递给节点的那份 YAML 里读出生效值**，不可能再分叉。
             '  · linefit 参数**已跟着槽位切**（生效值由 launch 运行时读该文件打印）：',
             _YamlKeysReadout(segmentation_params,
                              ('sensor_height', 'gravity_aligned_frame',
                               'input_topic', 'ground_output_topic')),
             '  · 自击掩膜（只作用于判据/限速，不动点云与 /segmentation/* 标签）：',
             _SelfMaskReadout(self_mask_params),
             '  · lio_tf_adapter：lio:=small_point_lio/cartographer/none 时**不启动**该节点',
             '（它由 LIO 自己用 TF 做 odom→base_link 的相似变换）。',
             _MountReadout()]))
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_LIO_cmd)
    ld.add_action(declare_nav_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_mapper_cmd)
    ld.add_action(declare_ground_cmd)
    ld.add_action(declare_global_obstacle_cmd)
    ld.add_action(declare_local_obstacle_cmd)
    # ★ 2026-10-06：「续建 + 场地隔离」相关（默认值下现有行为不变，详见各自的 description）
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_map_autocontinue_cmd)
    ld.add_action(declare_map_start_pose_cmd)
    ld.add_action(declare_map_allow_world_mismatch_cmd)
    ld.add_action(declare_cloud_accumulator_cmd)
    ld.add_action(declare_map_resume_check_cmd)
    ld.add_action(declare_map_resume_check_delay_cmd)
    ld.add_action(declare_map_resume_check_strict_cmd)
    ld.add_action(declare_map_resume_check_pos_tol_cmd)
    ld.add_action(declare_map_resume_check_yaw_tol_deg_cmd)
    ld.add_action(declare_map_session_announce_cmd)

    # ★ 2026-10-08：`model://` 本地解析根（= `robot:=robot11` 不再需要手工前缀 GAZEBO_MODEL_PATH）。
    #   ⚠️ **必须在 `start_rm_simulation` 之前**：它内部的 gzserver/gzclient launch 在**执行时**
    #   读 launch 进程的 `os.environ['GAZEBO_MODEL_PATH']`；本 Action 通过 `AppendEnvironmentVariable`
    #   改的正是这个 `os.environ`（launch 的 `context.environment` 就是 `os.environ`）。
    #   顺序反了 ⇒ mesh 解析根晚一步，gzclient 照样会回落到在线模型库并卡住（本 Task 的回归点）。
    ld.add_action(OpaqueFunction(function=_gazebo_model_path_setup))
    ld.add_action(start_rm_simulation)
    ld.add_action(bringup_imu_complementary_filter_node)
    # 地面分割槽位：两个节点都进 LaunchDescription，但各自的 condition 保证**只有一个**真的被启动
    #（⇒ /segmentation/obstacle 与 /segmentation/ground 恒定只有一个发布者）。
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(bringup_patchwork_ground_segmentation_node)
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
    # 两条互斥的适配器（默认模型 / robot:=hzmirm），同一时刻只有一个真的启动
    ld.add_action(lio_tf_adapter_node)
    ld.add_action(lio_tf_adapter_hzmirm_node)
    ld.add_action(lio_tf_adapter_robot11_node)
    # ★ 续建/从零的判定与横幅在 **t=0** 就执行（OpaqueFunction 是 Action，`--show-args` 不执行它）；
    #   slam_toolbox 节点本身仍由 OpaqueFunction 内部的 TimerAction 延后 4 s 起 ⇒ 时序与改造前一致。
    ld.add_action(start_mapping)
    ld.add_action(start_cloud_accumulator)
    ld.add_action(TimerAction(period=4.0, actions=[start_cartographer_mapping,
                                                   start_cartographer_as_lio_mapping]))
    ld.add_action(TimerAction(period=10.0, actions=[start_navigation2]))
    # 纯建图模式的 RViz（nav2 未启动时 nav_rviz 仍要能出图）
    ld.add_action(mapping_rviz_node)

    return ld
