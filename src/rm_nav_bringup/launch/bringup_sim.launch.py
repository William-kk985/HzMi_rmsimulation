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
    'scan_context_relocalizer': 'global_relocalizer:=none（默认，当前行为）或先建 DB：'
                                'ros2 run scan_context_relocalizer build_sc_db -- --bag <建图 bag/raw> '
                                '--world <world> --out <PCD/<world>_sc_db.npz>',
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
# global_relocalizer 槽位（2026-10-28）—— 全局重定位器（可选；默认关）
# -----------------------------------------------------------------------------
# 语义（契约 §4.16 / I3 / I4）：
#   · `none`（默认）= **当前行为逐字节不变**：不声明节点、不引入任何话题；
#   · `scan_context` = 起 scan_context_relocalizer：一帧激光 → SC keyframe DB → 闸门通过后
#     发 `/initialpose`（map 系粗位姿）+ `~/health`；**绝不发 TF**（map→odom 仍归 local tracker）。
#   · 与 `localization` **正交**：`localization:=gicp|small_gicp|icp|amcl|beluga|…` 都消费
#     `/initialpose`；槽位只决定"要不要有人替操作员点 2D Pose Estimate"。
#
# DB 路径 = `<rm_nav_bringup>/PCD/<world>_sc_db.npz`。**缺文件 = 响亮失败**：
#   `_WorldScDbFile` 在 Node.execute() 时才求值（--show-args / 不选该槽位 ⇒ 零成本），
#   报错里直接给建库命令 —— 与文件顶部 `_PackageShareFile` 的原则一致。
# =============================================================================
class _WorldScDbFile(Substitution):
    """按 `world` 惰性解析 `PCD/<world>_sc_db.npz`；缺文件时给出可操作的建库命令。"""

    def perform(self, context):
        world = (context.launch_configurations.get('world') or '').strip()
        if not world:
            raise RuntimeError('[launch] global_relocalizer:=scan_context 需要 world:=<name>（不能为空）')
        share = get_package_share_directory('rm_nav_bringup')
        path = os.path.join(share, 'PCD', '%s_sc_db.npz' % world)
        if not os.path.isfile(path):
            raise RuntimeError(
                "[launch] global_relocalizer:=scan_context 缺少 Scan Context DB：\n"
                "  期望：%(path)s\n"
                "  本次命令**显式选了** scan_context 槽 ⇒ 先离线建库（用该 world 的建图跑 bag）：\n"
                "    source /opt/ros/humble/setup.bash && source install/setup.bash\n"
                "    ros2 run scan_context_relocalizer build_sc_db -- \\\n"
                "      --bag <mapping run bag/raw> --world %(world)s \\\n"
                "      --pcd <rm_nav_bringup>/PCD/%(world)s.pcd \\\n"
                "      --out <workspace>/src/rm_nav_bringup/PCD/%(world)s_sc_db.npz\n"
                "    colcon build --symlink-install --packages-select rm_nav_bringup\n"
                "  （格式/命令/诚实边界见 docs/frame_contract_v3.md §4.16）\n"
                "  · 或者回退到默认：global_relocalizer:=none"
                % {'path': path, 'world': world})
        return path

    def describe(self):
        return '%s(world→PCD/<world>_sc_db.npz)' % type(self).__name__


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
    # ★ 2026-10-20：默认槽位改为**单刚体底盘**变体（生成物；几何与源文件逐条相等）。
    #   为什么（实测，见 docs/frame_contract_v3_measurements.md §20）：planar_move 把同一个
    #   (v, ω) 写给模型里的**每一个 link**，对多刚体 + 关节树不自洽 ⇒ 原 5 刚体/4 关节版本
    #   纯原地转只执行 **4.3%**（wz=0.8 → 实测 0.034 rad/s）。单刚体 + 轮子对地摩擦 0.02
    #   ⇒ **95.4%**；前进 80.9% → 88.1%（同一台车/世界/命令的台架实测）。
    #   源文件 `sentry_robot_sim.xacro` **一个字节都没改**：它仍是生成器的输入，
    #   也是 `--chassis articulated` 回退档的基准。
    '': ('sentry_robot_sim_rigid.xacro',
         '默认模型（单刚体底盘；几何 = urdf/sentry_robot_sim.xacro 逐条相等，'
         '雷达在 base_link+0.12,0,0.175）'),
    'hzmirm': ('sentry_robot_hzmirm_sim.xacro',
               '用户给的哨兵 URDF（雷达在云台头上 base_link+0,0,0.8；运动学逐字保留）'),
    'robot11': ('sentry_robot_robot11_sim.xacro',
                '用户的哨兵 robot11（SolidWorks 导出：真 mesh + 真 inertial；'
                '雷达在底盘上、斜 30° 下俯、离地 0.2595 m）'),
    'chassis': ('sentry_robot_chassis_sim.xacro',
                '用户的起伏底盘（4 轮、固定轮；真 mesh + 真 inertial；'
                '雷达**平放**在车顶、离地 0.188 m；IMU 平放）'),
    # ★ 2026-10-30：**诊断用**——与 chassis 几何/质量/插件完全相同，只把 <visual> 换成基本体
    #   （排查 gzclient 卡死是否由 STL 视觉件引起；不是正式槽位）。
    'chassis_boxvis': ('sentry_robot_chassis_boxvis_sim.xacro',
                       '诊断：起伏底盘 + 基本体视觉（排查 gzclient）'),
}

#: robot:=robot11 的雷达安装倾角 **不再有 launch 常数**（2026-10-21 收口，契约 §4.12）。
#: 唯一来源 = 上游 URDF 的 `body_to_livox` joint 的 `<origin rpy>` → 生成器
#: `tools/scripts/regress/robot11_make_sim_xacro.py` 把它烘进生成物
#: `urdf/sentry_robot_robot11_sim.xacro` 的 `<xacro:arg name="livox_tilt_rpy" default="..."/>`
#: （以及插件 `<tilt_rpy>` / `imu_joint` 的 rpy/xyz）。本 launch 只从**这次真要 spawn 的
#: xacro** 里读回它（`_robot11_tilt_rpy_from_xacro()`，只服务旧表回退/交叉校验）；
#: 运行期真值仍然由 `_parsed_livox_mount()` 展开关节链得到。要换倾角 ⇒ 改上游 URDF + 重跑
#: 生成器（scratch：`robot11_make_sim_xacro.py --livox-tilt-rpy "r p y"`），**不是**改这里。

#: ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已**删除**（不是"保留成硬报错别名"）。
#:   安装方式只剩**一种**（= 2026-10-21 起 `sensor` 的唯一语义）：`base_link→livox_frame`
#:   rpy=0（与实车 TF 一致、只有平移）、倾角在 `livox_frame→imu_link`（IMU 物理上真的斜）、
#:   插件 `<tilt_rpy>` 把**射线**旋斜并把**点**旋进水平的 `livox_frame`。这套语义已经**烘进
#:   生成物 xacro**（`<xacro:arg name="livox_mount" default="sensor"/>` + 三条常量 property）
#:   ⇒ 本 launch 不再传 `livox_mount`，xacro 取默认值 = **生成物逐字节不变**（measurements §38）。
#:   ⚠️ Humble 对未知的 `name:=value` **静默接受**（§28.4 实测）⇒ 传 `robot11_mount:=...`
#:   会在 t=0 被 `_check_removed_launch_args()` **硬拦下**（不静默忽略、也不静默等价）。


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


_ROBOT11_TILT_CACHE = {}


def _robot11_tilt_rpy_from_xacro():
    """robot11 的安装倾角（弧度字符串）—— 从**生成物** xacro 的 `livox_tilt_rpy` arg 默认值读回。

    这是 2026-10-21 收口后的**唯一** launch 侧来源（契约 §4.12）：倾角从上游 URDF 的
    `body_to_livox.rpy` 出发，由 `tools/scripts/regress/robot11_make_sim_xacro.py` 烘进
    `urdf/sentry_robot_robot11_sim.xacro` 的 `<xacro:arg .../>`，本函数只是把那个默认值读回来。
    **正常路径永远走 `_parsed_livox_mount()`**（展开这次真要 spawn 的 xacro、沿 fixed 关节链
    合成）；这里只给旧表回退/交叉校验（`_adapter_table()` / `_warn_adapter_drift()`）用，
    解析失败时表格也得有一个**独立于解析器**的倾角值，否则"回退"就没有意义。

    读不到 ⇒ 硬报错（不猜、不回退到某个字面值）。
    """
    if _ROBOT11_TILT_CACHE.get('v') is not None:
        return _ROBOT11_TILT_CACHE['v']
    import xml.etree.ElementTree as _et
    path = os.path.join(get_package_share_directory('rm_nav_bringup'), 'urdf',
                        _ROBOT_SLOTS['robot11'][0])
    try:
        doc = _et.parse(path).getroot()
    except Exception as e:                                   # noqa: BLE001
        raise RuntimeError(
            "[launch] 读不了 robot11 的生成物 %s 里的 `livox_tilt_rpy` arg：%s: %s。"
            "该文件由 tools/scripts/regress/robot11_make_sim_xacro.py 生成（倾角从上游 URDF 的 "
            "body_to_livox.rpy 烘进来）⇒ 请重新生成，**不要**在 launch 里手填角度。" % (path, type(e).__name__, e))
    for el in doc.iter('{http://ros.org/wiki/xacro}arg'):
        if el.get('name') == 'livox_tilt_rpy':
            v = (el.get('default') or '').strip()
            if len(v.split()) == 3:
                _ROBOT11_TILT_CACHE['v'] = v
                return v
            raise RuntimeError(
                "[launch] %s 的 `livox_tilt_rpy` 默认值不是 3 个数：%r（生成器坏了？）" % (path, v))
    raise RuntimeError(
        "[launch] %s 里没有 `livox_tilt_rpy` arg ⇒ 安装倾角没有单一来源（生成器/生成物不一致）"
        % path)


def _lio_adapter_robot11():
    """robot:=robot11 的 lio_tf_adapter 补偿量（xyz = base_link 原点在 imu 系下的坐标，rpy = 其姿态）。

    纯几何、无魔法：imu_link 的位姿 = body_to_livox(rpy) ∘ (0,0,-0.05)（imu_joint 是 rpy=0 的
    固定关节 ⇒ **IMU 跟着雷达一起动**，这正是让三份 LIO 配置的 extrinsic_T=[0,0,0.05]
    一个字节都不用改的原因）。T_imu←base_link 就是它的逆。

    ★ 2026-10-21（契约 §4.12 收口）：倾角**不再从 launch 常数读**，而是从生成物 xacro 的
    `livox_tilt_rpy` 默认值读（`_robot11_tilt_rpy_from_xacro()`）—— 那个值的源头是上游 URDF。
    `base_link→livox_frame` 现在 rpy=0、倾角只在 `livox_frame→imu_link`，所以这里**总是**用
    倾角构旋转。★ 2026-10-27（§4.15）：`mount` 参数已删除（安装方式不再是 launch 参数）。
    """
    tilt = [float(v) for v in _robot11_tilt_rpy_from_xacro().split()]
    rpy = tilt
    p_livox = (0.000561701465058485, 0.130915824456595, 0.15702816968305)
    R = _rpy_deg_to_R(rpy)                      # R_body←livox
    # imu 原点在 body 系 = p_livox + R·(0,0,−0.05)；姿态 = R
    off = [R[i][2] * -0.05 for i in range(3)]
    p_imu = [p_livox[i] + off[i] for i in range(3)]
    # T_imu←base_link：xyz = −Rᵀ·p_imu，rpy = R 的逆（= 同一个轴的负角）
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(Rt[i][j] * p_imu[j] for j in range(3)) for i in range(3)]
    rpy_out = [-v for v in rpy]
    return ([round(v, 9) for v in t], [round(v, 9) for v in rpy_out])


# =============================================================================
# URDF 安装参数解析器（2026-10-21）—— 从**这次真要 spawn 的 xacro** 读安装参数
# -----------------------------------------------------------------------------
# `_robot_xacro_args()` 同时喂 `robot_description` 与解析器（同一份参数，不许各写一套）。
# 从 `livox_frame`/`imu_link` 沿 child→parent 合成到 `base_link`（只允许 fixed 关节）：
#   T(base←livox) → gravity_align_rpy（度）+ gravity_align_h 的 livox_z（+地面偏移表）；
#   T(imu←base)   → lio_tf_adapter 的 xyz/rpy（**弧度**，含 livox→imu 的 0.05 m）。
# 缓存 (robot,mount,全部参数,urdf)；解析 vs 旧表差 >0.05° 或 >1 mm ⇒ WARN 但保留解析值；
# 已知模型解析失败 ⇒ WARN + 旧表回退；未知模型 ⇒ 硬报错。
# =============================================================================
import math as _math
import xml.etree.ElementTree as _ET

#: 解析值 vs 旧表的告警阈值（契约口径：0.05° / 1 mm）
_RPY_AXIS_TOL_DEG = 0.05
_XYZ_AXIS_TOL_M = 0.001

#: (robot, mount, xacro 参数, urdf 路径) → 解析结果 dict | None（None = 解析失败、已回退旧表）
_MOUNT_PARSE_CACHE = {}

#: linefit 每个槽位的参数文件（与 generate_launch_description 里 _RobotSlotFile 的 slot_map 同源；
#: 解析器只**读**它做交叉检查，不修改任何参数 —— 默认路径零改动）
_LINEFIT_SLOT_FILES = {
    '': ('config', 'segmentation_sim.yaml'),
    'hzmirm': ('config', 'segmentation_sim_hzmirm.yaml'),
    'robot11': ('config', 'segmentation_sim_robot11.yaml'),
    'chassis': ('config', 'segmentation_sim_chassis.yaml'),
}


class _MountParseError(RuntimeError):
    """URDF/xacro 解析失败（可回退旧表；与"用户参数写错"的 RuntimeError 区分开）。"""


class _MountUnitError(RuntimeError):
    """弧度参数里出现了 |值| > π（= 度/弧度混用的硬错误；不回退、不许静默变成 30 rad）。"""


def _assert_radians(what, values, where):
    """守卫：弧度参数的每个分量必须落在 ±π（+1e-6）内。

    为什么是硬错误（不是 WARN/回退）：`30.0`（度）当弧度进 `setRPY()` 会让
    `odom→base_link` 滚转 −110.7°、scan z −0.552、local costmap 几乎空（2026-10-21 实测踩到）。
    这类泄漏只会来自**单位换算写错**，必须在第一个消费者处停机，而不是"回退旧表"继续跑。
    """
    bad = [v for v in values if abs(v) > _math.pi + 1e-6]
    if bad:
        raise _MountUnitError(
            "[launch] %s 的取值超出 ±π：%s（rad）；where=%s。\n"
            "  这几乎一定是**度/弧度混用**：URDF <origin rpy>、TF、tf2 的 setRPY()/四元数都是"
            "**弧度**；表里的 -30° 必须只在**唯一一处**乘 M_PI/180。\n"
            "  ⇒ 拒绝继续（不会把 30 rad 静默写进 TF）；请检查 _R_to_rpy_rad() 的调用方与所有"
            "带 '°' 的变量。" % (what, values, where))


def _known_robot(context):
    """robot 槽位 → 已登记模型名；未登记 ⇒ 保持原来的硬报错（不回退、不猜）。"""
    robot = (LaunchConfiguration('robot').perform(context) or '').strip()
    if robot not in _ROBOT_SLOTS:
        raise RuntimeError(
            "[launch] 需要从 URDF 推出 `base_link→livox_frame` 的安装参数，但 robot:=%r "
            "这一档本 launch 没有它的安装几何（目前只有 ''|hzmirm|robot11）。"
            "请改用已知模型，或把该模型加进 _ROBOT_SLOTS（**不要**手填一个数兜底）" % robot)
    return robot


def _urdf_path_for_robot(robot):
    """robot 槽位 → 包 share 下的 xacro 绝对路径（与 _RobotSlot 选的是同一个文件）。"""
    return os.path.join(get_package_share_directory('rm_nav_bringup'), 'urdf',
                        _ROBOT_SLOTS[robot][0])


def _xacro_unquote(value):
    """去掉 measurement_params_sim.yaml 里为了让 shlex 不切分而写的**字面双引号**。"""
    v = str(value).strip()
    if len(v) >= 2 and v[0] == '"' and v[-1] == '"':
        return v[1:-1]
    return v


def _platform_livox_origin():
    """默认模型 xacro 的 `xyz`/`rpy` 实参（与 generate_launch_description 读的是**同一份 YAML**）。"""
    path = os.path.join(get_package_share_directory('hzmi_rm_simulation'), 'config',
                        'measurement_params_sim.yaml')
    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f)['base_link2livox_frame']
    return _xacro_unquote(d['xyz']), _xacro_unquote(d['rpy'])


def _robot_xacro_args(context, xyz, rpy):
    """**唯一真源**：robot_description 的 xacro 命令行参数列表。

    返回 [(name, value, needs_quote)]，顺序 = 原 command 字符串的顺序。
    `_RobotXacroCommand.perform`（真的 spawn 用）与 `_robot_xacro_mappings`（解析器用）都走这里
    ⇒ "解析的模型"与"spawn 的模型"在参数层面不可能分叉。改这里两边同时变。
    """
    robot = (LaunchConfiguration('robot').perform(context) or '').strip()
    # ★ 2026-10-30：起伏底盘的雷达平放在车顶（xyz=0 0 0.09 / rpy=0 0 0）——覆盖平台 yaml 的
    #   "默认模型"安装几何（0.12,0,0.175），否则 spawn 的模型与解析的模型会分叉。
    if robot in ('chassis', 'chassis_boxvis'):
        xyz, rpy = '0 0 0.09', '0 0 0'
    args = [('xyz', _xacro_unquote(xyz), True), ('rpy', _xacro_unquote(rpy), True)]
    # ★ 2026-10-27（契约 §4.15）：`robot:=hzmirm` 的 `turret_yaw_deg` / `turret_pitch_deg`
    #   **已删除** —— 云台是**静态**的（上游 URDF 里两个云台关节都是 fixed，本仓**没有**
    #   gimbal 控制器），这两个参数只改 spawn 时的静态姿态；默认 0/0 ⇒ 不再传，
    #   xacro 用默认 0/0，生成物逐字节不变。要试别的姿态请用 scratch xacro
    #   （见 docs/robot_models.md §3.3）。
    if robot == 'robot11':
        # ★ 2026-10-21（契约 §4.12 收口）：**不再传 `livox_tilt_rpy`** —— 安装倾角的唯一来源是
        #   上游 URDF，生成器把它烘进生成物 xacro 的 `<xacro:arg name="livox_tilt_rpy" .../>`
        #   默认值；这里再传一份就等于第二个所有者（已删除 `_LIVOX_TILT_RPY_ROLL`）。
        #   `livox_raise_m` 同样不再从 launch 传（xacro 默认 0 = 与上游几何逐字相同）。
        # ★ 2026-10-27（契约 §4.15）：`livox_mount` 也已**不再传** —— 安装方式只剩 `sensor` 一种，
        #   且已烘进 xacro（`<xacro:arg name="livox_mount" default="sensor"/>` + 常量 property）
        #   ⇒ 取默认值 = 同一个字符串，生成的 `robot_description` 逐字节不变（measurements §38）。
        args += [('visual_decimated',
                  'true' if (LaunchConfiguration('robot11_visual').perform(context) or '').strip()
                  == 'decimated' else 'false', False)]
    return args


def _render_robot_xacro_command(context, urdf_path, xyz, rpy):
    """把 `_robot_xacro_args` 渲染成 command 字符串。

    ★ 2026-10-21：robot11 的 `livox_raise_m:=0.0` 已删除（冻结 0），command 字符串因此比 1b5db4b
    少这一个参数；xacro 同默认值 ⇒ **生成的 robot_description 仍逐字节相同**（见 measurements §28）。
    ★ 2026-10-21（契约 §4.12 收口）：`livox_tilt_rpy:=...` 也已删除（倾角烘在生成物 xacro 的
    arg 默认值里）⇒ command 再少一个参数；xacro 取默认值 = 同一个字符串 ⇒ 输出仍逐字节相同
    （见 measurements §31）。
    """
    parts = ['xacro ', urdf_path]
    for name, value, needs_quote in _robot_xacro_args(context, xyz, rpy):
        parts += [' ', name, ':=', ('"%s"' % value) if needs_quote else value]
    return ''.join(parts)


def _robot_xacro_mappings(context):
    """解析器要传给 `xacro.process_file` 的 mappings（= 上面那份参数，去掉引号）。"""
    xyz, rpy = _platform_livox_origin()
    return {name: value for name, value, _ in _robot_xacro_args(context, xyz, rpy)}


def _expand_urdf_xml(path, mappings):
    """展开 xacro → XML 文本。返回 (xml_text, source)。

    两级（够用且不引第三份机制）：
      ① in-process `xacro.process_file` —— 与 launch 真 spawn 的 `xacro` CLI **同一个库**、
         同一份 mappings（`_robot_xacro_mappings`）；空文档视为失败。
      ② 仅当文件**本来就是 plain URDF** 时直接读 XML（含 xacro 构造 ⇒ 明确失败，不猜）。
    ⚠️ 不再单独起 `xacro` CLI 子进程：CLI 依赖的就是这个 Python 模块，两级已覆盖本仓槽位；
    删掉它是为了不把"解析器"变成一个自带进程管理的子系统（复杂度审查，2026-10-21）。
    """
    errors = []
    try:
        import xacro
        doc = xacro.process_file(path, mappings=dict(mappings))
        xml_text = doc.toxml() if hasattr(doc, 'toxml') else str(doc)
        if not xml_text.strip():
            raise RuntimeError('xacro returned an empty document')
        return xml_text, 'xacro.process_file'
    except Exception as e:                                   # noqa: BLE001 - 只用于回退
        errors.append('xacro.process_file: %s: %s' % (type(e).__name__, e))
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
        if 'xacro:' in text or '<xacro' in text:
            errors.append('plain-XML fallback not applicable: file still contains xacro constructs')
        else:
            return text, 'plain XML (URDF)'
    except Exception as e:                                   # noqa: BLE001
        errors.append('read plain XML: %s: %s' % (type(e).__name__, e))
    raise _MountParseError('cannot expand %s: %s' % (path, ' | '.join(errors)))


def _T_from_origin(origin_el, label):
    """`<origin xyz rpy>`（URDF 约定：R = Rz(yaw)·Ry(pitch)·Rx(roll)）→ 4x4。"""
    xyz = [0.0, 0.0, 0.0]
    rpy_rad = [0.0, 0.0, 0.0]
    if origin_el is not None:
        for attr, dst in (('xyz', xyz), ('rpy', rpy_rad)):
            raw = (origin_el.get(attr) or '').split()
            if raw:
                if len(raw) != 3:
                    raise _MountParseError('%s: <origin %s="%s"> 不是 3 个数'
                                           % (label, attr, ' '.join(raw)))
                dst[:] = [float(v) for v in raw]
    R = _rpy_deg_to_R(rpy_rad)          # 该函数名里的 deg 是历史误名：参数是**弧度**
    return [[R[0][0], R[0][1], R[0][2], xyz[0]],
            [R[1][0], R[1][1], R[1][2], xyz[1]],
            [R[2][0], R[2][1], R[2][2], xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _round9(v):
    """round 9 位，并把 -0.0 归一成 +0.0（数值相同，避免 YAML/日志里出现 -0.000000 的假漂移）。"""
    return round(v, 9) + 0.0


def _mat4_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _mat3_transpose(R):
    return [[R[j][i] for j in range(3)] for i in range(3)]


def _R_to_rpy_rad(R):
    """旋转矩阵 → URDF rpy（**弧度**），与 `_rpy_deg_to_R` 的 Rz·Ry·Rx 约定互逆。"""
    sy = _math.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        roll = _math.atan2(R[2][1], R[2][2])
        pitch = _math.atan2(-R[2][0], sy)
        yaw = _math.atan2(R[1][0], R[0][0])
    else:                       # 万向锁：roll/yaw 只有组合有意义，取 roll=0 的标准解
        roll = 0.0
        pitch = _math.atan2(-R[2][0], sy)
        yaw = _math.atan2(-R[0][1], R[1][1])
    return [roll, pitch, yaw]


def _R_to_rpy_deg(R):
    """旋转矩阵 → rpy（**度**）；只用于 `gravity_align_rpy`（LIO 参数是度）。"""
    return [_math.degrees(v) for v in _R_to_rpy_rad(R)]


def _compose_fixed_chain(xml_text, target_link, root_link, label):
    """沿 target→root 的关节树合成 T(root ← target)，只允许 fixed 关节。

    返回 (4x4, chain)；chain = [{'joint','type','child','parent'}, ...]（从 target 往上）。
    """
    try:
        doc = _ET.fromstring(xml_text)
    except _ET.ParseError as e:
        raise _MountParseError('%s: 展开后的 XML 解析失败：%s' % (label, e))
    links = {el.get('name') for el in doc.findall('link')}
    for name in (target_link, root_link):
        if name not in links:
            raise _MountParseError(
                '%s: 解析出的 URDF 里没有 link %r（目标/根帧）；现有 links=%s'
                % (label, name, sorted(n for n in links if n)[:20]))
    child2joint = {}
    for joint in doc.findall('joint'):
        parent_el, child_el = joint.find('parent'), joint.find('child')
        if parent_el is None or child_el is None:
            continue
        child2joint[child_el.get('link')] = (parent_el.get('link'), joint.get('name') or '?',
                                            (joint.get('type') or '').strip(), joint.find('origin'))
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    chain, seen, cur = [], set(), target_link
    while cur != root_link:
        if cur in seen:
            raise _MountParseError('%s: 关节树成环（link %r 重复出现）' % (label, cur))
        seen.add(cur)
        entry = child2joint.get(cur)
        if entry is None:
            raise _MountParseError(
                '%s: 从 %r 往上找不到父关节（断树 / 根不是 %r）；已走链路=%s'
                % (label, cur, root_link, ' → '.join([root_link] + [c['child'] for c in reversed(chain)])))
        parent, jname, jtype, origin_el = entry
        chain.append({'joint': jname, 'type': jtype, 'child': cur, 'parent': parent})
        if jtype != 'fixed':
            raise _MountParseError(
                "%s: 安装链上出现**非 fixed** 关节 %r（type=%s）：%s → %s。"
                "本解析器只合成 fixed 关节（不猜运动学）：请把该安装链改成 fixed，"
                "或先把它固定到该跑姿态再把角度写进 URDF；**不要**在 launch 里手填。"
                % (label, jname, jtype, parent, cur))
        T = _mat4_mul(_T_from_origin(origin_el, '%s joint %r' % (label, jname)), T)
        cur = parent
    return T, chain


def _parse_mount_from_urdf(urdf_path, mappings, robot='?', mount='?'):
    """展开 + 解析。返回 dict（rpy_deg/xyz/livox_z/adapter_*/chain/source/urdf...）。"""
    xml_text, source = _expand_urdf_xml(urdf_path, mappings)
    label = '%s (robot=%r mount=%r)' % (urdf_path, robot, mount)
    T_livox, chain = _compose_fixed_chain(xml_text, 'livox_frame', 'base_link', label)
    R = [row[:3] for row in T_livox[:3]]
    p = [T_livox[i][3] for i in range(3)]
    out = {
        'robot': robot, 'mount': mount, 'urdf': urdf_path, 'source': source,
        'rpy_deg': [_round9(v) for v in _R_to_rpy_deg(R)],
        'xyz': [_round9(v) for v in p],
        'livox_z': p[2],
        'chain': chain,
        'adapter_xyz': None, 'adapter_rpy': None, 'imu_chain': None, 'imu_error': None,
    }
    # IMU 链（adapter 的 T(imu←base)）；IMU 不在链上时不让整次解析失败（h/rpy 仍然可用）
    try:
        T_imu, imu_chain = _compose_fixed_chain(xml_text, 'imu_link', 'base_link', label)
        R_imu = [row[:3] for row in T_imu[:3]]
        p_imu = [T_imu[i][3] for i in range(3)]
        Rt = _mat3_transpose(R_imu)                       # R(imu←base) = R(base←imu)ᵀ
        t = [-sum(Rt[i][j] * p_imu[j] for j in range(3)) for i in range(3)]
        out['adapter_xyz'] = [_round9(v) for v in t]
        out['adapter_rpy'] = [_round9(v) for v in _R_to_rpy_rad(Rt)]   # ⚠️ 弧度：lio_tf_adapter 的 rpy 进 setRPY()
        _assert_radians('T(imu_link←base_link) 的 rpy', out['adapter_rpy'], label)
        out['imu_chain'] = imu_chain
        # ★ 2026-10-21（契约 §4.12）：**重力先验现在来自 IMU 链**（`base_link→imu_link`）。
        #   为什么：`base_link→livox_frame` 已经 rpy=0（与实车 TF 一致），倾角只存在于 IMU 链上；
        #   而 `odom` 是 IMU 系的原始世界 ⇒ R_grav = R(base_link←imu_link)。
        out['gravity_rpy_deg'] = [_round9(v) for v in _R_to_rpy_deg(R_imu)]
        # LIO 的 `extrinsic_R/T` = T(imu←livox)：p_imu = R_IL·p_lidar + T_IL。
        #   `extrinsic_T` 仍然是 [0,0,0.05]（= 雷达原点在 IMU 系里的位置：沿两者共同的 z 偏 5 cm）；
        #   `extrinsic_R` = R(imu←livox) = 倾角的**逆**（水平点云 → 斜的 IMU 系）。
        # ★ 2026-10-30：IMU 与 livox 可能是**兄弟**（都挂在 base_link 下，如起伏底盘）⇒ 直接
        #   向上走链会失败；此时用两条 base 链合成 T(livox←imu) = T(base←livox)^-1 · T(base←imu)。
        try:
            T_il, _ilk = _compose_fixed_chain(xml_text, 'imu_link', 'livox_frame', label)  # T(livox←imu)
        except _MountParseError:
            # T(base←livox) 的逆
            R_l = [row[:3] for row in T_livox[:3]]
            p_l = [T_livox[i][3] for i in range(3)]
            R_lt = _mat3_transpose(R_l)
            p_lt = [-sum(R_lt[i][j] * p_l[j] for j in range(3)) for i in range(3)]
            T_livox_inv = [[R_lt[i][0], R_lt[i][1], R_lt[i][2], p_lt[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]
            T_il = _mat4_mul(T_livox_inv, T_imu)   # T(livox←imu) = T(livox←base)·T(base←imu)
        R_li = [row[:3] for row in T_il[:3]]
        p_li = [T_il[i][3] for i in range(3)]
        R_il = _mat3_transpose(R_li)
        t_il = [-sum(R_il[i][j] * p_li[j] for j in range(3)) for i in range(3)]
        out['extrinsic_xyz'] = [_round9(v) for v in t_il]
        out['extrinsic_rpy_rad'] = [_round9(v) for v in _R_to_rpy_rad(R_il)]
        out['extrinsic_R'] = [_round9(v) for row in R_il for v in row]     # 行主序 9 个
    except _MountParseError as e:
        out['imu_error'] = str(e)
        out['gravity_rpy_deg'] = None
        out['extrinsic_xyz'] = None
        out['extrinsic_R'] = None
        out['extrinsic_rpy_rad'] = None
    return out


def _urdf_livox_mount_table(context):
    """**旧规则表**（1b5db4b 及以前 launch 里那张）：只作交叉校验与解析失败时的回退。"""
    robot = _known_robot(context)
    if robot == 'robot11':
        # ★ 2026-10-21（契约 §4.12）：`base_link→livox_frame` 现在**只有平移**（倾角搬到
        #   `livox_frame→imu_link`）⇒ 这张旧表的 rpy 变成 0（交叉校验用；倾角见 gravity_rpy_deg）。
        # ★ 2026-10-27（§4.15）：`robot11_mount` 已删除 —— 安装方式只剩 `sensor` 一种，
        #   这里不需要再校验什么（几何完全由 URDF 决定）。
        rpy = [0.0, 0.0, 0.0]
        xyz = [0.000561701465058485, 0.130915824456595, 0.15702816968305]
    elif robot == 'hzmirm':
        # ★ 2026-10-27（契约 §4.15）：`turret_yaw_deg` / `turret_pitch_deg` 已删除 ⇒ 云台
        #   **固定**在默认姿态 yaw=0 / pitch=0（= 上游 URDF 的数值）。原来那三行按 yaw/pitch
        #   构造 R 的代码在 0/0 时就是单位阵 ⇒ 这里直接写下等价常量：
        #   R = I ⇒ xyz = [0, 0, 0.6 + 0.2] = [0, 0, 0.8]，rpy = [0, 0, 0]。
        #   （云台是**静态**的：上游两个关节都是 fixed，本仓没有 gimbal 控制器。）
        rpy = [0.0, 0.0, 0.0]
        xyz = [0.0, 0.0, 0.8]
    elif robot in ('chassis', 'chassis_boxvis'):
        # 起伏底盘：雷达平放在车顶（z 0.09），与 sentry_robot_chassis_sim.xacro 的一致。
        rpy = [0.0, 0.0, 0.0]
        xyz = [0.0, 0.0, 0.09]
    else:                                   # 默认模型：平台外参 YAML = xacro 的 xyz/rpy 实参
        xyz_s, rpy_s = _platform_livox_origin()
        xyz = [float(v) for v in xyz_s.split()]
        rpy = [float(v) for v in rpy_s.split()]
        if len(xyz) != 3 or len(rpy) != 3:
            raise RuntimeError('[launch] measurement_params_sim.yaml 的 base_link2livox_frame '
                               '不是 3+3 个数：xyz=%r rpy=%r' % (xyz_s, rpy_s))
    return {'rpy_deg': [_round9(v) for v in rpy], 'xyz': [_round9(v) for v in xyz],
            'livox_z': xyz[2]}


def _adapter_table(context):
    """旧表的 adapter 补偿量（T(imu←base)），解析失败时回退。"""
    robot = _known_robot(context)
    if robot == 'robot11':
        return _lio_adapter_robot11()
    if robot == 'hzmirm':
        return [0.0, 0.0, -0.75], [0.0, 0.0, 0.0]
    if robot in ('chassis', 'chassis_boxvis'):
        return [0.0, 0.0, -0.04], [0.0, 0.0, 0.0]
    return [-0.12, 0.0, -0.125], [0.0, 0.0, 0.0]


def _linefit_yaml_path(robot):
    rel = _LINEFIT_SLOT_FILES.get(robot)
    if rel is None:
        return None
    try:
        share = get_package_share_directory('linefit_ground_segmentation_ros')
    except PackageNotFoundError:
        return None
    path = os.path.join(share, *rel)
    return path if os.path.isfile(path) else None


def _read_sensor_height(path):
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == 'sensor_height' and isinstance(v, (int, float)):
                    return float(v)
                found = walk(v)
                if found is not None:
                    return found
        elif isinstance(node, list):
            for v in node:
                found = walk(v)
                if found is not None:
                    return found
        return None
    return walk(data)


def _check_linefit_sensor_height(robot, h_parsed):
    """交叉检查（**只 WARN、不改任何参数**）：linefit YAML 的 sensor_height vs 解析出的 h。

    ⚠️ 本 launch 从不给 linefit 传 sensor_height（它按 robot 槽位选整份 YAML）；两份都是
    "雷达离地高度"的副本 ⇒ 这里只负责让它**不能悄悄漂**：URDF 一改，YAML 没跟上就报。
    """
    path = _linefit_yaml_path(robot)
    if path is None:
        return
    try:
        yaml_h = _read_sensor_height(path)
    except Exception as e:                                   # noqa: BLE001
        print('[bringup_sim][WARN] linefit 参数交叉检查读不了 %s：%s' % (path, e), flush=True)
        return
    if yaml_h is None or abs(yaml_h - h_parsed) <= 0.01:
        return
    print('[bringup_sim][WARN] linefit sensor_height 与 URDF 解析高度不一致：'
          'yaml=%.6f m vs parsed h=%.6f m（Δ=%+.4f m）← %s。'
          '本 launch **不改**该 YAML（避免第二真源/默认路径变化），请同步它。'
          % (yaml_h, h_parsed, yaml_h - h_parsed, path), flush=True)


def _fmt_vec(v):
    return '[%s]' % ', '.join('%.6f' % x for x in v)


def _angle_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def _warn_mount_drift(parsed, table, robot, mount, path):
    """解析值 vs 旧表：>0.05°/轴 或 >1 mm/xyz ⇒ 响亮 WARN，并**保留解析值**。"""
    d_rpy = [_angle_diff_deg(p, t) for p, t in zip(parsed['rpy_deg'], table['rpy_deg'])]
    d_xyz = [p - t for p, t in zip(parsed['xyz'], table['xyz'])]
    if (max(abs(v) for v in d_rpy) <= _RPY_AXIS_TOL_DEG
            and max(abs(v) for v in d_xyz) <= _XYZ_AXIS_TOL_M):
        return
    print('[bringup_sim][WARN] URDF 安装参数与旧表不一致（**保留解析值** = 这次真的 spawn 的模型）：\n'
          '  robot=%r mount=%r urdf=%s（source=%s）\n'
          '  rpy(parsed)=%s° vs rpy(table)=%s°  Δ=%s°\n'
          '  xyz(parsed)=%s m vs xyz(table)=%s m  Δ=%s m\n'
          '  ⇒ 表只作交叉校验；若解析值才是实物，请同步 docs/frame_contract_v3.md §13 '
          '与 linefit 的 sensor_height。'
          % (robot, mount, path, parsed['source'],
             _fmt_vec(parsed['rpy_deg']), _fmt_vec(table['rpy_deg']), _fmt_vec(d_rpy),
             _fmt_vec(parsed['xyz']), _fmt_vec(table['xyz']), _fmt_vec(d_xyz)), flush=True)


def _warn_adapter_drift(parsed, robot, mount, path, context):
    """解析出的 T(imu←base) vs 旧表（阈值同 0.05°/1 mm）：不一致 ⇒ WARN。

    ⚠️ `lio_tf_adapter` 的 `rpy` 参数是**弧度**（C++ `q_offset.setRPY(rpy_[0], ...)`），
    而 `gravity_align_rpy` 是**度** —— 两个参数单位不同，别把解析器的度直接喂给 adapter。

    为什么连 adapter 也要比：`lio_tf_adapter_hzmirm_node`（mode=off 的 legacy 非-spl 路径）
    仍用字面值 `[0,0,-0.75]` —— 本函数保证"URDF 改了但那个字面值没跟"时**至少是响的**。
    """
    if parsed.get('adapter_xyz') is None:
        return
    try:
        t_xyz, t_rpy = _adapter_table(context)
    except Exception:                                        # noqa: BLE001 - 只是交叉检查
        return
    d_rpy = [_angle_diff_deg(_math.degrees(p), _math.degrees(t))
             for p, t in zip(parsed['adapter_rpy'], t_rpy)]
    d_xyz = [p - t for p, t in zip(parsed['adapter_xyz'], t_xyz)]
    if (max(abs(v) for v in d_rpy) <= _RPY_AXIS_TOL_DEG
            and max(abs(v) for v in d_xyz) <= _XYZ_AXIS_TOL_M):
        return
    print('[bringup_sim][WARN] adapter 的 T(imu<-base_link) 与旧表不一致（**保留解析值**）：\n'
          '  robot=%r mount=%r urdf=%s\n'
          '  xyz(parsed)=%s m vs xyz(table)=%s m  Δ=%s m\n'
          '  rpy(parsed)=%s rad vs rpy(table)=%s rad  Δ=%s°\n'
          '  ⇒ mode!=off 的 spl adapter 用解析值；legacy `lio_tf_adapter_hzmirm_node` 若仍写字面值，'
          '请同步它（这是"URDF 改了、字面值没跟"的告警）。'
          % (robot, mount, path,
             _fmt_vec(parsed['adapter_xyz']), _fmt_vec(t_xyz), _fmt_vec(d_xyz),
             _fmt_vec(parsed['adapter_rpy']), _fmt_vec(t_rpy), _fmt_vec(d_rpy)), flush=True)


def _parsed_livox_mount(context, urdf_path=None, mappings=None, robot=None, mount=None,
                        use_cache=True):
    """解析入口（带缓存/交叉校验/回退）。

    正常路径（launch 用）：`_parsed_livox_mount(context)`。
    测试路径（决定性 30°→20°）：传 `urdf_path=` / `mappings=` / `robot=` / `mount=` 指向 scratch 副本。
    返回：解析 dict；解析失败（已知模型）⇒ 已打 WARN，返回 None（调用方回退旧表）。
    """
    robot = robot if robot is not None else _known_robot(context)
    if robot not in _ROBOT_SLOTS:
        raise RuntimeError("[launch] robot:=%r 不在 _ROBOT_SLOTS 里" % robot)
    if mount is None:
        # ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已删除 ⇒ `mount` 现在只是**日志/缓存标签**
        #   （安装方式只剩 xacro 里烘死的一种）。不再读任何 launch 参数，也不会失败。
        mount = 'sensor' if robot == 'robot11' else ''
    if mappings is None:
        mappings = _robot_xacro_mappings(context)          # 配置错误（轴/挂法非法）不吞、直接抛
    if urdf_path is None:
        urdf_path = _urdf_path_for_robot(robot)
    key = (robot, mount, tuple(sorted(mappings.items())), urdf_path)
    if use_cache and key in _MOUNT_PARSE_CACHE:
        return _MOUNT_PARSE_CACHE[key]
    try:
        parsed = _parse_mount_from_urdf(urdf_path, mappings, robot=robot, mount=mount)
        # 防御式再查一次：解析器内部的 guard 是主防线；这里保证"任何 _parse_mount_from_urdf
        # 实现返回的 adapter rpy"都必须是弧度（测试会 monkeypatch 它来模拟度泄漏）。
        if parsed.get('adapter_rpy') is not None:
            _assert_radians('T(imu_link←base_link) 的 rpy', parsed['adapter_rpy'],
                            'robot=%r mount=%r' % (robot, mount))
    except _MountUnitError:
        raise                                            # 单位 bug = 硬错误，绝不回退成"能跑"
    except Exception as e:                                   # noqa: BLE001 - 只回退，不掩盖参数错
        if use_cache:
            _MOUNT_PARSE_CACHE[key] = None
        print('[bringup_sim][WARN] 从 URDF 解析安装参数失败 ⇒ **本次回退旧表**（不是静默）：\n'
              '  robot=%r mount=%r urdf=%s\n  原因：%s: %s\n'
              '  ⇒ rpy/xyz/h 用 _urdf_livox_mount_table()/表内几何；请修好 xacro 或 URDF。'
              % (robot, mount, urdf_path, type(e).__name__, e), flush=True)
        return None
    # ★ 启动日志：度 + 弧度并排，肉眼可查（单位 bug 的现场证据）；缓存 ⇒ 每条 run 只打一次。
    print('[bringup_sim] URDF mount T(base_link<-livox_frame): robot=%r mount=%r '
          'rpy=%s° = %s rad, xyz=%s m, livox_z=%.9f, source=%s'
          % (robot, mount,
             _fmt_vec(parsed['rpy_deg']),
             _fmt_vec([_math.radians(v) for v in parsed['rpy_deg']]),
             _fmt_vec(parsed['xyz']), parsed['livox_z'], parsed['source']), flush=True)
    # 解析成功：与旧表交叉校验（只 WARN，不改变返回值 = 解析值）
    try:
        table = _urdf_livox_mount_table(context)
    except Exception as e:                                   # noqa: BLE001
        table = None
        print('[bringup_sim][WARN] 解析成功但旧表交叉校验不可用：%s: %s' % (type(e).__name__, e),
              flush=True)
    if table is not None:
        _warn_mount_drift(parsed, table, robot, mount, urdf_path)
        off = _HZMI_BASE_LINK_HEIGHT_M.get(robot)
        if off is not None:
            _check_linefit_sensor_height(robot, round(off + parsed['livox_z'], 9))
    _warn_adapter_drift(parsed, robot, mount, urdf_path, context)
    if use_cache:
        _MOUNT_PARSE_CACHE[key] = parsed
    return parsed


# =============================================================================
# ★ 2026-10-21（契约 v3 §4.14）：`gravity` —— 重力/水平帧参数组的**唯一用户开关**
# -----------------------------------------------------------------------------
# 用户决定（原话）："123都可以感觉raw也可以留着可以学cod以baselink为基准"。
# 四个取值 = 旧 4 个参数（level_frame × gravity_align_mode × gravity_align_z_origin ×
# z_origin_source）的**组合**，一次选完：
#
#   gravity:=tf（默认）
#       odom = **原始**（IMU）世界（E1 off）；局部代价图/静态边帧 = `odom_level`
#       （唯一发布者 odom_level_tf、latched；z=0 = 地面，值从 URDF 的 h 算）；
#       E1 关；z 原点 = ground/urdf；障碍层 = 今天（local_obstacle 的 2D scan / stvl）。
#   gravity:=base
#       odom = **原始**（IMU）世界（E1 off）；**没有** odom_level 边、局部代价图 = odom；
#       高度语义放在 **base_link**（p2l target_frame=base_link、linefit gravity_aligned_frame=
#       base_link）；局部障碍层强制**逐点**（cloud | stvl）—— 2D scan 平盘在斜 odom 里没有
#       逐点高度可言（§25 的 43.7% 半盲）。显式 local_obstacle:=scan ⇒ **硬报错**。
#   gravity:=lio
#       odom = **水平**（E1 hybrid；只对 lio:=small_point_lio，其它 LIO 硬报错）；
#       局部代价图 = odom；z 原点 = ground（E1b）；障碍层 = 今天。
#   gravity:=raw
#       odom = **原始**（IMU）世界（E1 off）；局部代价图 = odom；障碍层 = 今天的 off 档
#       （2D scan 允许 = 复现 §25 的"半盲"臂）。
#
# 旧参数（gravity_align_mode / gravity_align_z_origin / level_frame / z_origin_source /
# lio_init_settle_s / lio_init_map_extrinsic）**只作为硬报错别名保留**：传非默认值 ⇒ t=0 停，
# 消息指向 gravity（见 `_check_gravity_contract` / `_LEGACY_GRAVITY_ALIASES`）。
# =============================================================================
_GRAVITY_MODES = ('tf', 'base', 'lio', 'raw')
_GRAVITY_DEFAULT = 'tf'


def _gravity(context):
    """`gravity` 的取值（tf | base | lio | raw）—— 容错读（未声明/取不到 ⇒ 默认档）。"""
    try:
        raw = str(context.launch_configurations.get('gravity', _GRAVITY_DEFAULT)).strip()
    except Exception:                                              # noqa: BLE001
        raw = _GRAVITY_DEFAULT
    raw = raw or _GRAVITY_DEFAULT
    if raw not in _GRAVITY_MODES:
        raise RuntimeError("[launch] gravity:=%r 不是可用取值（%s）—— 选错不静默回退"
                           % (raw, ' | '.join(_GRAVITY_MODES)))
    return raw


class _Gravity(Substitution):
    """把 `gravity` 喂给下游（node 参数 / condition / 合并层）。惰性求值。"""

    def perform(self, context):
        return _gravity(context)

    def describe(self):
        return '%s()' % type(self).__name__


def _level_frame(context):
    """派生：水平帧（局部代价图 / 静态边 / rviz）—— 只有 `gravity:=tf` 才有 `odom_level`。"""
    return 'odom_level' if _gravity(context) == 'tf' else 'odom'


class _LevelFrame(Substitution):
    """把派生的水平帧喂给下游（node 参数 / condition / 合并层）。

    惰性求值（同 `_Gravity` 的原则：不读这个槽位的 launch 组合零成本）。
    """

    def perform(self, context):
        return _level_frame(context)

    def describe(self):
        return '%s()' % type(self).__name__


def _launch_arg_was_given(context, name):
    """用户是否**显式**给了这个 launch 参数（只在命令行 argv 里找）。

    Humble 的 `ros2 launch` 把 `name:=value` 原样放进 `LaunchContext.argv`；而 launch 的
    默认值与显式值在 `launch_configurations` 里**无法区分**。父 include 给的 launch_arguments
    不可见 ⇒ 按默认处理（这是本仓"尽力而为"的边界，已写进 §4.14 的诚实清单）。
    """
    try:
        argv = context.argv or []
    except Exception:                                              # noqa: BLE001
        return False
    prefix = name + ':='
    return any(str(a).startswith(prefix) for a in argv)


#: ★ 2026-10-27（契约 §4.15）：三个**已删除**的 launch 参数 → 给用户的替代说明。
#: 为什么还要这张表：ROS 2 Humble 对**未知**的 `name:=value` 是**静默接受**的（§28.4 实测）——
#: 删掉 `DeclareLaunchArgument` 只是让 `--show-args` 里不再出现，传了却**不会**报错。
#: 本表 + `_check_removed_launch_args()` 把"静默忽略"变成 **t=0 硬报错**。
_REMOVED_LAUNCH_ARGS = (
    ('robot11_mount',
     '`robot:=robot11` 的安装方式只剩**一种**（= 2026-10-21 起的 `sensor` 语义），已烘进生成物 '
     'xacro 的 `<xacro:arg name="livox_mount" default="sensor"/>`；launch 不再需要也不再接受它。'),
    ('turret_yaw_deg',
     '`robot:=hzmirm` 的云台是**静态**的（上游两个关节都是 fixed、本仓**没有** gimbal 控制器），'
     '现在固定 yaw=0° = 原来的默认值。'),
    ('turret_pitch_deg',
     '`robot:=hzmirm` 的云台是**静态**的（上游两个关节都是 fixed、本仓**没有** gimbal 控制器），'
     '现在固定 pitch=0° = 原来的默认值；`h = 0.6 + 0.2·cos(0) = 0.8 m` 不变。'),
)


def _check_removed_launch_args(context):
    """★ 2026-10-27（契约 §4.15）：三个**已删除** launch 参数的 t=0 硬报错守卫。

    检测面（两条，任一命中就停）：
      ① `context.launch_configurations`：这三个名字**没有** `DeclareLaunchArgument` ⇒ 没有默认值
         ⇒ 它们出现在这里 = 有人显式设了（命令行、或父 include 的 `launch_arguments`；
         launch 的 include 会把 `SetLaunchConfiguration` 先执行、再访问子 LD）；
      ② `context.argv`（`_launch_arg_was_given()`，与 gravity 的旧别名同口径）：命令行原文。
         ①已覆盖②；保留②是为了"即使将来有人给它们加回声明（带默认值），命令行显式传仍然会被抓"。

    为什么不用"声明成硬报错别名"（像 §4.14 的 6 个 gravity 旧名）：本任务要求这三项从
    `--show-args` 里**消失**（§4 的验收），而声明了就一定还在。

    ⚠️ 边界（诚实清单）：若父 launch 自己在**它自己的作用域**里声明了同名参数、又不传给本
    launch，本文件看不到"来源"——但那样它也不会进本 context 的 `launch_configurations`
    （除非父 include 真的传了 `launch_arguments`，那正是我们要拦的）。
    """
    given = []
    for name, hint in _REMOVED_LAUNCH_ARGS:
        value = context.launch_configurations.get(name)
        from_argv = _launch_arg_was_given(context, name)
        if value is None and not from_argv:
            continue
        if value is None:                                  # 只在 argv 里出现（防御式，不猜值）
            value = '<见命令行 argv>'
        given.append('  · %s:=%s\n      %s' % (name, value, hint))
    if given:
        raise RuntimeError(
            "[launch] 这三个 launch 参数**已删除**（契约 §4.15 / measurements §38），"
            "但本命令仍然传了：\n%s\n"
            "  ⇒ 直接去掉它们重跑即可（默认行为 = 唯一行为）：\n"
            "     · `robot11_mount`：xacro 默认 `sensor`，生成物与传 `robot11_mount:=sensor` 逐字节相同；\n"
            "     · `turret_yaw_deg` / `turret_pitch_deg`：hzmirm 云台固定 0/0（静态 spawn 姿态，"
            "没有控制器），`h = 0.8 m` 不变。\n"
            "  历史与证据：docs/frame_contract_v3.md §4.15 + "
            "docs/frame_contract_v3_measurements.md §38"
            % '\n'.join(given))
    return []


#: `gravity:=base` 允许的局部障碍层 = **逐点**层（带逐点高度语义）。
_PER_POINT_LOCAL_OBSTACLE = ('cloud', 'stvl')


def _effective_local_obstacle(context):
    """局部障碍层的**有效值**（`gravity:=base` 时强制逐点层）。

    为什么 base 档不能留 2D scan：局部代价图帧 = `odom`（LIO 的**原始**世界，IMU 斜装就斜 30°），
    而 `/scan` 是 2D 平盘 —— nav2 的 `obstacle_layer` 先 `projectLaser`（z 置 0）再搬到代价图帧
    ⇒ 每条波束的 z 恒等于那一帧传感器原点的 z，**没有逐点高度**可判（§25 实测 in-band 43.7%、
    kept-y 单侧 = 半盲）。逐点云（cloud / stvl）才有"离 base_link 多高"的语义。
    """
    raw = (context.launch_configurations.get('local_obstacle', 'scan') or 'scan').strip() or 'scan'
    if _gravity(context) != 'base':
        return raw
    if raw in _PER_POINT_LOCAL_OBSTACLE:
        return raw
    if raw == 'scan' and _launch_arg_was_given(context, 'local_obstacle'):
        raise RuntimeError(
            "[launch] gravity:=base 与 local_obstacle:=scan **不能组合**：\n"
            "  gravity:=base 的局部代价图帧是 `odom`（LIO 的**原始**世界，IMU 斜装就斜 30°），"
            "而 `/scan` 是 2D 平盘 —— nav2 的 obstacle_layer 先 projectLaser（z 置 0）再搬到"
            "代价图帧 ⇒ 每条波束的 z 恒等于那一帧传感器原点的 z，**没有逐点高度**可判。\n"
            "  实测代价（measurements §25 的 off 档）：in-band 只有 43.7%、kept-y 单侧 = 半盲。\n"
            "  ⇒ 用逐点层：local_obstacle:=cloud|stvl；或换 gravity:=tf（odom_level 水平帧）")
    if raw == 'scan':
        return 'cloud'
    if raw == 'both':
        return 'cloud'
    if raw == 'stvl_both':
        return 'stvl'
    return raw


class _EffectiveLocalObstacle(Substitution):
    """把局部障碍层的有效值喂给 nav2 include / scan 高度带覆盖层。"""

    def perform(self, context):
        return _effective_local_obstacle(context)

    def describe(self):
        return '%s()' % type(self).__name__


#: 三个 LIO 槽位的 `extrinsic_T`（= **livox 原点在 IMU body 系里的位置**），三份配置**同值**：
#:   small_point_lio/config/mid360_sim.yaml、point_lio/config/pointlio_mid360_sim.yaml、
#:   FAST_LIO/config/fastlio_mid360_sim.yaml 都是 `[0.0, 0.0, 0.05]`。
#: 静态边的 z 原点要用它把"livox 离地高度 h"换成"状态原点（IMU）离地高度 h_imu"（契约 §4.7 的公式）。
#: ⚠️ 改任何一份 LIO 配置的 extrinsic_T 都必须同步这里（否则 odom_level 的 z=0 会偏 5 cm 量级）；
#:    运行期 `odom_level_tf` 会把它打进日志，`gravity_align_h_check` 会暴露偏差。
_LEVEL_EXTRINSIC_T = (0.0, 0.0, 0.05)


def _level_tf_geometry(context):
    """静态边 `odom → odom_level` 的三个**输入事实**：(input_world, input_z, output_z)。

    只在 `gravity:=tf` 时被使用（`odom_level_tf` 的 condition）。`gravity:=tf` ⇒ E1 关、
    z 原点固定 = ground/urdf（§4.14）：

    · `input_world`：喂进来的 odom 是否**已经**重力对齐？
        `tilted` = 原始 body 世界（E1 关：SPL-off / fastlio）⇒ 旋转部分 = `R_mount`；
        `level`  = 已经对齐（pointlio 的 Set_init / none / cartographer）⇒ 旋转 = I；
    · `input_z`：喂进来的 odom 的 z=0 在哪？（`sensor` = 状态原点=IMU 原点；`ground` = 地面）；
    · `output_z`：`odom_level` 的 z=0 要放在哪（`gravity` 的 z 原点约定，tf ⇒ `ground`）。

    ⇒ `t_level = (0, 0, h_imu)` 当 `input_z=sensor` 且 `output_z=ground`，否则 `(0,0,0)`
      （`h_imu = h − (R_mount·t_IL)_z`，见 `odom_level_tf.py`）。
    """
    lio = _Nav2ParamsForSlot._cfg(context, 'lio') or 'fastlio'
    if lio in ('none', 'cartographer'):
        # 这两个槽位不接管 odom→base_link：静态边做成**恒等别名**（odom_level ≡ odom）
        # ⇒ 局部代价图/consumer 只换帧名、**零行为变化**，也不对它们的世界系做任何重力假设。
        return ('level', 'ground', 'ground')
    if lio == 'pointlio':
        # Point-LIO 的 `Set_init` 已经把世界摆正（ImuProcess::Set_init），但 z 原点在 IMU
        return ('level', 'sensor', 'ground')
    if lio == 'small_point_lio' and _effective_gravity_align_mode(context) != 'off':
        # 只有 gravity:=lio 会到这里（那时 level_frame=odom、本节点根本不启动）；保留成恒等，
        # 防止以后有人把 E1 臂接回静态边时"再转一次"。
        return ('level', 'ground', 'ground')
    # SPL-off / fastlio（以及未来"原始 body 世界"的 LIO）：世界 = 初始 body 系（IMU 斜装就斜）
    return ('tilted', 'sensor', 'ground')


class _LevelTfParam(Substitution):
    """`odom_level_tf` 的一个参数值（`input_world` / `input_z` / `output_z`）。"""

    def __init__(self, which):
        super().__init__()
        self.__which = which

    def perform(self, context):
        iw, iz, oz = _level_tf_geometry(context)
        if self.__which == 'input_world':
            return iw
        if self.__which == 'input_z':
            return iz
        if self.__which == 'output_z':
            return oz
        raise RuntimeError('[launch] _LevelTfParam(which=%r) 未知' % self.__which)

    def describe(self):
        return '%s(%s)' % (type(self).__name__, self.__which)


def _level_tf_receipt(context):
    """`gravity:=tf` 的 t=0 收据（一行，把"谁发哪条边、各帧是哪来的"写死）。"""
    lio = _Nav2ParamsForSlot._cfg(context, 'lio') or 'fastlio'
    iw, iz, oz = _level_tf_geometry(context)
    return ('[odom_level] gravity=tf ⇒ 帧树 = map→odom→odom_level→base_link；'
            '静态边 odom→odom_level 的**唯一发布者** = odom_level_tf（latched，值由 URDF 的 '
            'gravity_align_rpy + gravity_align_h 算出）；lio=%s；'
            'input_world=%s input_z=%s output_z=%s；adapter 父帧=odom_level、'
            'local_costmap.global_frame=odom_level；换档 = gravity:=base|lio|raw'
            % (lio, iw, iz, oz))


class _LevelTfReceipt(Substitution):
    """`LogInfo` 的消息体（惰性求值 ⇒ `--show-args` 不执行、不读参数）。"""

    def perform(self, context):
        return _level_tf_receipt(context)

    def describe(self):
        return '%s()' % type(self).__name__


#: ★ 2026-10-20：E1 重力对齐的档位（**实验臂已收口**：只有 `gravity:=lio` 会用 hybrid）。
#: 这些值只作为旧参数 `gravity_align_mode` 的 choices（硬报错别名）保留。
_GRAVITY_ALIGN_MODES = ('off', 'urdf', 'imu', 'hybrid', 'auto')


def _effective_gravity_align_mode(context):
    """`gravity` ⇒ E1 的**有效档**（`off | hybrid`）—— **唯一的解析点**。

    · `tf | base | raw` ⇒ `off`（tf 的重力对齐由静态边 `odom→odom_level` 承担；base/raw 不做）；
    · `lio` ⇒ `hybrid` —— **只对 `lio:=small_point_lio`**（E1 补丁只存在于那个包；其它 LIO
      硬报错，与旧 §4.10 的"显式 hybrid + 非 SPL ⇒ t=0 停"同一语义）。

    ⚠️ 有效档只给**下游**（condition / 节点参数 / 收据）用；旧 `gravity_align_mode` 参数
    不再参与任何解析（传非默认值 ⇒ 硬报错，见 `_check_gravity_contract`）。
    """
    if _gravity(context) != 'lio':
        return 'off'
    lio = (LaunchConfiguration('lio').perform(context) or '').strip()
    if lio != 'small_point_lio':
        raise RuntimeError(
            "[launch] gravity:=lio 只对 lio:=small_point_lio 生效（E1 初始化对齐的补丁在 "
            "small_point_lio 包里；fastlio/pointlio/cartographer 没有 gravity_align_mode / "
            "gravity_align_rpy 这些参数，传了也不会生效）⇒ 当前 lio:=%s：请改成 "
            "lio:=small_point_lio，或换 gravity:=tf|base|raw" % (lio or '(空)'))
    return 'hybrid'


class _GravityAlignEffectiveMode(Substitution):
    """把 E1 的有效档喂给下游（condition / 节点参数）。

    惰性求值（照抄 `_GravityAlignRpy` / `_PackageShareFile` 的原则：不读这个槽位的 launch 组合
    零成本）。返回值永远是 `off|hybrid` —— 旧 `auto/urdf/imu` 不再存在。
    """

    def perform(self, context):
        return _effective_gravity_align_mode(context)

    def describe(self):
        return '%s()' % type(self).__name__


def _gravity_receipt(context):
    """`gravity` 的 t=0 收据（一行；把 4 个值各自的 odom/帧/E1/z/障碍层写死）。"""
    g = _gravity(context)
    lio = (LaunchConfiguration('lio').perform(context) or '').strip() or '(空)'
    eff = _effective_gravity_align_mode(context)
    level = _level_frame(context)
    obstacle = _effective_local_obstacle(context)
    z_origin = {'tf': 'ground(urdf)', 'base': 'n/a', 'lio': 'ground', 'raw': 'ground'}[g]
    odom = 'level(E1 hybrid)' if g == 'lio' else 'raw(IMU)'
    return ('[gravity] gravity=%s ⇒ odom=%s、局部代价图/静态边帧=%s、E1=%s、z 原点=%s、'
            '局部障碍层=%s（lio=%s）%s'
            % (g, odom, level, eff, z_origin, obstacle, lio,
               '' if level == 'odom_level' else
               '；**没有 odom_level 这个帧**（odom_level_tf 不启动）'))


class _GravityReceipt(Substitution):
    """`LogInfo` 的消息体（惰性求值 ⇒ `--show-args` 不执行、不读参数）。"""

    def perform(self, context):
        return _gravity_receipt(context)

    def describe(self):
        return '%s()' % type(self).__name__


#: 旧参数 = **硬报错别名**：`(name, 旧默认值, 替代说明)`。
#: 传的值 != 旧默认值 ⇒ t=0 停（消息指向 gravity）。显式传回旧默认值无法与"没传"区分
#: （launch 不保留来源），按默认处理 —— 这是本条规则的已知边界（§4.14 诚实清单）。
_LEGACY_GRAVITY_ALIASES = (
    ('gravity_align_mode', 'auto',
     'gravity:=tf|base|lio|raw（lio ⇒ E1 hybrid；tf/base/raw ⇒ E1 off）'),
    ('gravity_align_z_origin', 'ground',
     'gravity:=tf（ground/urdf 固定）；gravity:=lio（E1b ground 固定）'),
    ('level_frame', 'odom_level',
     'gravity:=tf ⇒ odom_level；gravity:=base|lio|raw ⇒ odom'),
    ('z_origin_source', 'urdf',
     'gravity:=tf（z 原点来源固定 = URDF 的 h，确定、可跨跑比较）'),
    ('lio_init_settle_s', '1.0',
     '无参数：初始化**不再等落定**（17e468d 起出生即落定；门已删除）'),
    ('lio_init_map_extrinsic', 'True',
     '无参数：初始化地图**一律**用 R_IL·p + t_IL（与逐点更新同口径）'),
)


def _check_gravity_contract(context):
    """★ 2026-10-21（§4.14）：`gravity` 的**组合检查 + t=0 收据**（OpaqueFunction）。

    1. 旧参数硬报错别名（传非默认值 ⇒ 停，消息指向 gravity）；
    2. `gravity:=lio` 只对 `lio:=small_point_lio`（在 `_effective_gravity_align_mode` 里抛）；
    3. `gravity:=base` 的局部障碍层强制逐点（显式 `local_obstacle:=scan` ⇒ 停）；
    4. `gravity:=tf` + `localization:=''` 的全局/局部帧不一致 WARN（§4.11 的既有告警）。
    """
    bad = []
    for name, default, hint in _LEGACY_GRAVITY_ALIASES:
        raw = context.launch_configurations.get(name)
        if raw is None:
            continue
        if str(raw).strip() != default:
            bad.append('  · %s:=%s（旧默认值 = %s）⇒ 改用 %s'
                       % (name, str(raw).strip(), default, hint))
    if bad:
        raise RuntimeError(
            "[launch] 旧参数已删除（硬报错别名）—— 重力/水平帧只有**一个**用户开关："
            "gravity:=tf|base|lio|raw\n%s\n"
            "  契约 §4.14 / measurements §33；`ros2 launch … --show-args` 可看 gravity 的完整语义"
            % '\n'.join(bad))

    actions = []
    # 3) base 档的障碍层：先求值（非法组合在这里就抛），再打 WARN（静默降级不允许）
    eff_obstacle = _effective_local_obstacle(context)
    if _gravity(context) == 'base':
        raw_obstacle = (context.launch_configurations.get('local_obstacle', 'scan')
                        or 'scan').strip() or 'scan'
        if eff_obstacle != raw_obstacle:
            msg = ('[launch][WARN] gravity=base ⇒ 局部障碍层 local_obstacle:=%s 被强制成 %s'
                   '（2D /scan 在斜的 odom 里没有逐点高度；§25 实测 in-band 43.7%% = 半盲）'
                   % (raw_obstacle, eff_obstacle))
            print(msg, flush=True)
            actions.append(LogInfo(msg=msg))

    # 4) tf 档 + LIO 当绝对定位（localization:=''）的回退告警（§4.11 的既有语义）
    if _level_frame(context) == 'odom_level':
        mode = _Nav2ParamsForSlot._cfg(context, 'mode') or 'mapping'
        loc = _Nav2ParamsForSlot._cfg(context, 'localization')
        lio = _Nav2ParamsForSlot._cfg(context, 'lio') or 'fastlio'
        if mode in ('nav', 'slam_nav') and loc == '' and lio in (
                'fastlio', 'pointlio', 'small_point_lio'):
            msg = ('[launch][WARN] gravity=tf + localization=\'\'（LIO 当绝对定位）：'
                   '静态桥把 map≡odom 钉成恒等，而 odom 是 %s 的**原始**世界（可能斜装）⇒ '
                   '全局代价图（map）不会跟着摆正，只有局部（odom_level）是正的。'
                   '正式用法：给一个重定位槽（localization:=gicp|amcl|slam_toolbox|…）；'
                   '或换 gravity:=base|lio|raw' % lio)
            print(msg, flush=True)
            actions.append(LogInfo(msg=msg))

    # 1/2 的 t=0 收据（也负责触发 lio 组合的硬报错：_gravity_receipt 里会解析有效档）
    actions.insert(0, LogInfo(msg=[_GravityReceipt()]))
    if _level_frame(context) == 'odom_level':
        actions.append(LogInfo(msg=[_LevelTfReceipt()]))
    elif _effective_gravity_align_mode(context) != 'off':
        # gravity:=lio 的 E1 收据：把"用哪个先验、h 多少、谁发 TF"写死（与旧 §4.10 的收据同义）
        rpy = _urdf_livox_mount_rpy_deg(context)
        h = _gravity_align_h(context)
        actions.append(LogInfo(msg=(
            '[gravity_align] gravity=lio ⇒ E1 hybrid（统一输出契约）：LIO publish_tf:=false + '
            'odom_convention:=sensor、lio_tf_adapter 成为 odom→base_link 的唯一发布者；'
            'gravity_align_rpy（来源=%s）=[%.6f, %.6f, %.6f]°；z_origin=ground gravity_align_h='
            '%.6f m（odom 的 z=0 = 初始化时刻的地面）'
            % (_mount_source_label(context), rpy[0], rpy[1], rpy[2], h))))

    # ★ 2026-10-27（契约 §4.12.5）：LIO 外参**键名**收据。放在 t=0、与 gravity 收据同一批：
    #   把"这个后端读哪两个键、期望值、谁做运行期断言"写死；`--show-args` 不执行。
    lio_sel = _Nav2ParamsForSlot._cfg(context, 'lio') or 'fastlio'
    if lio_sel in _LIO_EXTRINSIC_KEYS:
        actions.append(LogInfo(msg=[_LioExtrinsicReceipt()]))
    return actions


def _urdf_livox_mount_rpy_deg(context):
    """`base_link→livox_frame` 的 rpy（度）—— **从这次真的要 spawn 的 xacro 里解析出来的**。

    这就是 E1 的先验 `gravity_align_rpy`（"车体水平时雷达在世界里的姿态"）。**唯一事实来源是
    URDF**：`_parsed_livox_mount()` 用与 `robot_description` **同一份** xacro 参数展开模型，
    再沿 fixed 关节链合成 T(base_link←livox_frame) ⇒ 改 URDF（或加机器人）launch 自动跟着变。

    旧规则表（`_urdf_livox_mount_table()`）**仍然保留**，但只做两件事：
      ① 与解析值交叉校验：差 >0.05°/轴（或 >1 mm/xyz）⇒ 响亮 WARN，**保留解析值**（它才是真模型）；
      ② 已知模型解析失败时回退 + WARN（坏 parser 不许挡住所有 launch）。
    未知模型仍然**硬报错**（`_known_robot`），不回退、不猜。

    ⚠️ 这里**只**回答"URDF 说雷达相对车体是什么姿态"，不回答"URDF 与实物是否一致"——
    后者是 `gravity_align_mode:=hybrid` 的 Δ 互检（≥5° ⇒ ERROR + 改用 IMU）负责的事。
    """
    parsed = _parsed_livox_mount(context)
    if parsed is not None and parsed.get('gravity_rpy_deg') is not None:
        return parsed['gravity_rpy_deg']
    if parsed is not None:
        # IMU 链缺失（没有 imu_link??）⇒ 退回 livox rpy 并**响亮报**：重力先验会错
        print('[bringup_sim][WARN] 解析器没拿到 base_link→imu_link（%s）⇒ gravity_align_rpy 退回 '
              'base_link→livox_frame 的 rpy=%s°（按实车约定那一条现在是 0，重力先验会错！）'
              % (parsed.get('imu_error'), _fmt_vec(parsed['rpy_deg'])), flush=True)
        return parsed['rpy_deg']
    return _urdf_livox_mount_table(context)['rpy_deg']


def _mount_source_label(context):
    """banner 用：这次 rpy/xyz/h 到底来自**解析**（哪个 source）还是**旧表回退**。"""
    parsed = _parsed_livox_mount(context)
    if parsed is not None:
        return 'URDF-%s' % parsed['source']
    return 'table-fallback'


class _GravityAlignRpy(Substitution):
    """`gravity_align_rpy` 参数：**YAML 列表字符串**（launch_ros 会 yaml.safe_load 成 double 数组）。

    取值来自 `_urdf_livox_mount_rpy_deg()`（= URDF），不是这里写的常数 —— 这一层只是把
    "URDF 里的安装关节"翻译成 small_point_lio 的参数格式，**不含任何手填数值**。
    """

    def perform(self, context):
        v = _urdf_livox_mount_rpy_deg(context)
        return '[%s]' % ', '.join(repr(float(x)) for x in v)

    def describe(self):
        return '%s()' % type(self).__name__


#: 各模型 **base_link 原点离地高度**（m）—— E1b `gravity_align_h` 里**唯一不是 URDF 能直接给**的量。
#: 轮子/车体几何都在 URDF 里，但"URDF 摆到 Gazebo 地面上时 base_link 原点离地多高"= 轮半径 +
#: 轮心高度（以及轮胎下沉，见下）⇒ 保留一张**小的、逐台有证据**的表，作为先验。
#: `_gravity_align_h()` = 本表 + **解析出的 livox_z**（后者随 URDF 自动变，不再是手填常数）。
#: 证据（全部来自本仓文件）：
#:   · robot11 0.102499：`urdf/sentry_robot_robot11_sim.xacro` 的「地面平面（body 系）
#:     z = -0.102499」（轮 r=0.058、轮心 z=-0.044502 ⇒ 0.044502+0.058=0.102502，与 -0.102499
#:     差 3 µm = mesh 离散误差）；`robot11_weld_chassis.py` 合成刚体自检同样报“最低点 -0.10250 m”。
#:   · hzmirm 0.0：`base_to_wheel_*` 的 origin z=0.08、轮 cylinder 半径=0.08 ⇒ 轮底 = base_link z=0。
#:   · 默认模型 0.06：`radius_wheel=0.06`、`base_to_wheel*` origin z=0 ⇒ 轮底 z=-0.06。
#: ⚠️ 与实物/Gazebo 的差（轮胎下沉）不在这张表里：默认模型 linefit 的 0.226 vs 0.235 就是 ~9 mm
#:    下沉；robot11 的 0.2595 与几何 0.259527 差 0.03 mm（几乎无下沉）。运行时由节点初始化时的
#:    **地面拟合互检**负责（|h_meas−h_urdf| ≥0.03/0.05 m ⇒ WARN/ERROR，见契约 v3 §4.3 与
#:    `small_point_lio/src/gravity_align_ground.cpp`）；本 launch 只提供先验，不重复那段逻辑。
_HZMI_BASE_LINK_HEIGHT_M = {'robot11': 0.102499, 'hzmirm': 0.0, '': 0.06, 'chassis': 0.098, 'chassis_boxvis': 0.098}

#: 出生高度在"落定高度"之上再加的余量（m）。为什么不是 0：`spawn_entity.py` 把模型按**几何**
#: 摆到 `base_link_height + margin`，留 1 mm 让接触在第一个物理步里就解出来，而不是一出生就
#: 穿透地面（穿透会弹一下，等于又制造了一个小下落）。为什么不是 0.2（原值）：见 `_SpawnZForRobot`。
_HZMI_SPAWN_Z_MARGIN_M = 0.001

#: `floor z == 0` 的标定世界（`docs/worlds.md` §3：底板顶面 z=-1.6413436 m == world z 0）。
#: 只有这些 world 才把 spawn_z 改成"落定高度"；别处（RMUC/RMUL 2024 等）保持 world 配置的 z。
#: ⚠️ `get_world_config()` 的 z **不动**：它是 `map_asset_guard.WORLD_SPAWN` 的镜像源，也是既有
#: 存档 sidecar 的 `spawn_pose.z` 比对基准 ⇒ 改它会让既有 RMUL2026/RMUC2026 存档全部 `refused_spawn`。
_HZMI_SPAWN_AT_SETTLED_WORLDS = ('RMUC2026', 'RMUL2026', 'RMUC2027', 'RMUL2027')

#: 各 world **出生点脚下**的地面高度（world 系, m）。2026 两个 world 的出生点在 z=0 的地板上；
#: 2027 两个的出生候选落在**局部抬高**的地面上（`docs/worlds_2027.md` §6.2 / §9.4 的落板实测）：
#:   · RMUC2027 (5.90, 2.72)：地面 **+0.030 m**（0.2 m 方块落到底面 +0.0299）
#:   · RMUL2027 (−0.85, +0.05)：地面 **+0.010 m**（10 mm 中央垫高层）
#: 不含这一项 ⇒ spawn_z 会把车**埋进**这 1–3 cm 的地面里（1992.6 的物理步会把它弹出来 = 又制造
#: 了一次小下落，正是 `_SpawnZForRobot` 要消掉的东西）。默认值表里没有的 world ⇒ 0.0（= 旧行为）。
_HZMI_SPAWN_GROUND_Z = {'RMUC2026': 0.0, 'RMUL2026': 0.0, 'RMUC2027': 0.030, 'RMUL2027': 0.010}

#: **每个 world 的 mapper 参数槽**（2026-10-29，meas §45.2 / §46.3）。
#: 为什么需要它：`slam_toolbox` 的 `minimum_travel_distance=0.2` 是为 2026 场地调出来的
#: （`docs/slam_toolbox_tuning.md`），但 2027 的**格栅围栏**（2 m 立柱 + 3 根 5 cm 横杆）在
#: "上次修正 + 里程计"先验下**沿栅栏方向没有约束**：karto 只在 ±0.5 m/±20° 的窗口里匹配最近
#: ~10 个节点（4 Hz ⇒ 0.5–0.8 m 基线），0.2 m 的节点间距让基线短到贴着同一段栅栏 ⇒
#: 每帧允许的 0.1–0.2 m 滑动被下一帧继承，`map→base` 最终跑掉 2.19 m/28.4°（§45.2c 的 A 档）。
#: §46.3 先把节点间距放到 0.6 m/0.5 rad（G 档），位置误差回到 0.047 m 中位（0.158 max）。
#:
#: ★ 2026-10-30（meas §48）：用户要求"改参数试一试"**弱回环** ⇒ 以同一条 bag / 同一个节点做
#:   确定性 A/B（B1 = §46.3 的 0.6/0.5 现状 vs C2 = candidate）。**C2 赢**（10 项，见下）：
#:   0.6/0.5 → 0.5/0.3 节点密度 + 回环搜索窗 3→15 m、链 10→5、回环细匹配 0.45→0.60、
#:   顺序细匹配 0.1→0.3、scan_buffer 10→20、correlation 窗 0.5→0.3、loop 搜索窗 8→5 m、
#:   map_update_interval 5.0→1.5。实测：POS p90 0.156→0.141、max 0.158→0.143、刚性
#:   0.085→0.069 m、YAW max 5.11→4.68°；2D 图占用格 371→457、GT 墙召回 19.3→23.5%、
#:   STL 边沿覆盖 11.1→14.0%（POS 中位 0.046→0.051 m 是唯一变差项，+5 mm）。
#: ★ 任务书里的 `loop_search_space_resolution: 0.1` **没有采纳**：karto 2.6.10 要求
#:   smear_deviation ∈ [0.5·resolution, 10·resolution]，0.1 + 0.03 会在节点构造时 throw
#:   "Smear deviation too small" 直接 abort（§48 的 C1 档）。补 0.06 可跑（C3 档），但与 0.05
#:   逐字节同结果 ⇒ 保留参数文件的 0.05（不引入第 11 个改动）。
#: ★ 默认**只对 2027 生效**；2026 不在表里 ⇒ 不传任何覆盖 ⇒ 参数文件 + 节点构造**逐字节不变**。
#: ★ 仍**不**采用 §45.2c H 档的 `odom_frame:=odom_level`：那会让 slam_toolbox 发 map→odom_level，
#:   与静态边 odom→odom_level 冲突（两个父节点，§45.7#2）。
_MAPPER_2027_SLOT = {
    # 节点密度（§46.3 的 0.6/0.5 → candidate 的 0.5/0.3）
    'minimum_travel_distance': 0.5,
    'minimum_travel_heading': 0.3,
    # 回环：搜索半径 / 链长 / 细匹配门（§48 实测：本 bag 无真回环 ⇒ 未触发，只验证了不崩）
    'loop_search_maximum_distance': 15.0,
    'loop_match_minimum_chain_size': 5,
    'loop_match_minimum_response_fine': 0.6,
    'loop_search_space_dimension': 5.0,
    # 顺序匹配：缓冲区 / 细匹配门 / 相关搜索窗（§48 的实测差别主要来自这三项 + 节点密度）
    'scan_buffer_size': 20,
    'link_match_minimum_response_fine': 0.3,
    'correlation_search_space_dimension': 0.3,
    # 图刷新（5.0 → 1.5 s；CPU 无 measurable 变化，见 §48）
    'map_update_interval': 1.5,
    # ★ 2026-10-30（用户"改纯 LIO 的 odom 建图试一试"，§49）：**关掉扫描匹配 + 回环**，
    #   让 slam_toolbox 退化成"用 LIO 的 odom 栅格化器"——没有 map→odom 的离散修正
    #   （坡上/围栏旁的"突然一跳"消失），2D 图 = LIO odom 的精度（§45.2 实测 0.335 m/8.1°）。
    #   代价：绕大圈会累积 LIO 漂移（连续、不跳）。只对 2027 生效，2026 不受影响。
    'use_scan_matching': False,
    'do_loop_closing': False,
}
_MAPPER_PARAMS_BY_WORLD = {
    'RMUC2027': dict(_MAPPER_2027_SLOT),
    'RMUL2027': dict(_MAPPER_2027_SLOT),
}


class _SpawnZForRobot(Substitution):
    """`rm_simulation.launch.py` 的 `spawn_z`：**该模型落定时的 base_link 高度 + 1 mm**。

    ★ 2026-10-21 新增（契约 §4.13.5 / measurements §32）。动机：
      `rm_simulation.launch.py::get_world_config()` 的 `z=0.2` 对所有模型/所有 world 都是同一个
      常数，而三个模型的落定高度各不相同（robot11 0.1025 / hzmirm 0.0 / 默认 0.06）⇒ 车一出生
      就自由落体（robot11 9.75 cm，约 1.8 s），而 LIO 的初始化**不看车停没停**：
        · `pointlio` 在首帧后 **0.5 s** 就 `Set_init`（车还在下沉）⇒ odom 的 z 锚点低 6.5 cm；
        · `small_point_lio` 的 200 帧 IMU 门在 2.0 s（车 1.7 s 落定），但它的初始化地图会吞进
          整段下沉过程的点云 ⇒ 靠 §30 的落定门 + 初始化地图口径才兜住。
      直接从**出生高度**上消掉这个下落 ⇒ 两个 LIO 都在静止的车上初始化，锚点确定性 = URDF。
    ⚠️ 未知 robot / 未标定 world ⇒ 返回 ''（= 回落到 world 配置的 z，逐字节回到改造前）。
    """

    def perform(self, context):
        world = (LaunchConfiguration('world').perform(context) or '').strip()
        if world not in _HZMI_SPAWN_AT_SETTLED_WORLDS:
            return ''
        robot = (LaunchConfiguration('robot').perform(context) or '').strip()
        off = _HZMI_BASE_LINK_HEIGHT_M.get(robot)
        if off is None:
            return ''
        ground = _HZMI_SPAWN_GROUND_Z.get(world, 0.0)
        return '%.6f' % (ground + off + _HZMI_SPAWN_Z_MARGIN_M)

    def describe(self):
        return '%s(settled_worlds=%s, margin=%.3f)' % (
            type(self).__name__, ','.join(_HZMI_SPAWN_AT_SETTLED_WORLDS), _HZMI_SPAWN_Z_MARGIN_M)


def _gravity_align_h_table(context):
    """**旧表**推出的 h（解析失败时的回退；数值与 1b5db4b 的 `_gravity_align_h` 逐位相同）。"""
    robot = _known_robot(context)
    off = _HZMI_BASE_LINK_HEIGHT_M.get(robot)
    if off is None:
        raise RuntimeError(
            "[launch] gravity_align_h 缺 robot:=%r 的**地面偏移**（base_link 原点离地）：请把该模型"
            "加进 _HZMI_BASE_LINK_HEIGHT_M（用轮半径+轮心高度从它的 URDF 推，**不要**手填一个凑数的值）"
            % robot)
    return round(off + _urdf_livox_mount_table(context)['livox_z'], 9)


def _gravity_align_h(context):
    """传感器（`livox_frame` 原点）离地高度 h（米）—— E1b `gravity_align_h` 的先验。

    h = **base_link 原点离地高度**（`_HZMI_BASE_LINK_HEIGHT_M`：URDF 给不了的那一个量，逐台有证据）
      + **livox_frame 原点在 base_link 里的 z**（**解析**出来的 T(base_link←livox_frame)，随 URDF 变）

    逐模型（默认姿态）：
      · `robot:=robot11`：0.102499 + 0.15702816968305 = **0.259527 m**
        （与 linefit 的 `sensor_height: 0.2595` 交叉一致；`sensor` 档点云真的在传感器系）
      · `robot:=hzmirm`（云台 pitch=0）：0.0 + (0.3+0.3+0.2·cos(pitch)) = **0.800000 m**
        （= linefit 的 0.80；解析器会自己走 base→turret→head→lidar→livox 的关节链）
      · 默认模型（`robot:=''`）：0.06 + 0.175 = **0.235000 m**
        （linefit 的 `sensor_height: 0.226` 是 Gazebo 实测值 = 轮胎下沉 ~9 mm；URDF 几何值是 0.235）

    ⚠️ 这里只回答"URDF 说雷达在哪"，**不是**实测值。`livox_raise_m` 已在 2026-10-21 从 launch
    删除（冻结 0；xacro 仍保留同名 arg，默认 0 ⇒ 默认生成的 URDF 逐字节不变）。若要复现 B 方案，
    只能改 scratch 生成器/xacro；同时 linefit 的 `sensor_height` 必须手工同步（本 launch 不替改）。
    """
    robot = _known_robot(context)
    off = _HZMI_BASE_LINK_HEIGHT_M.get(robot)
    if off is None:
        raise RuntimeError(
            "[launch] gravity_align_h 缺 robot:=%r 的**地面偏移**（base_link 原点离地）：请把该模型"
            "加进 _HZMI_BASE_LINK_HEIGHT_M（用轮半径+轮心高度从它的 URDF 推，**不要**手填一个凑数的值）"
            % robot)
    parsed = _parsed_livox_mount(context)
    if parsed is not None:
        return round(off + parsed['livox_z'], 9)
    return _gravity_align_h_table(context)


class _GravityAlignH(Substitution):
    """`gravity_align_h` 参数：标量（米）= 地面偏移表 + **解析出的 livox_z**（不是常数）。"""

    def perform(self, context):
        return repr(float(_gravity_align_h(context)))

    def describe(self):
        return '%s()' % type(self).__name__


class _MountExtrinsic(Substitution):
    """LIO 的 `extrinsic_T` / `extrinsic_R`（= T(imu←livox)），**从这次 spawn 的 URDF 解析**。

    为什么要由 launch 传（契约 §4.12）：按实车约定，`base_link→livox_frame` 的 rpy = 0，
    倾角只存在于 `livox_frame→imu_link` 上 ⇒ 点云是**水平**的、IMU 系是**斜**的，
    两者之间那 30° 必须由 LIO 的外参吸收（`p_imu = extrinsic_R·p_lidar + extrinsic_T`）。
    三份 LIO 配置里的 `extrinsic_R: identity` 现在是**非 robot11 槽位的默认值**；
    robot11 由这里覆盖成 `Rx(+30°)`（= 倾角逆矩阵，行主序 9 个数），`extrinsic_T` 仍是
    `[0,0,0.05]`（两个系共同的 z 方向偏 5 cm，与改动前逐位相同）。

    ⚠️ 与 `_LioAdapterExtrinsic` 的区别：那个是 **T(imu←base_link)**（adapter 用来把
    LIO 的 body 位姿换成 base_link 位姿）；这个是 **T(imu←livox_frame)**（喂给估计器）。
    """

    def __init__(self, which):
        super().__init__()
        self.__which = which

    def perform(self, context):
        parsed = _parsed_livox_mount(context)
        if parsed is not None and parsed.get('extrinsic_R') is not None:
            xyz, rot9 = parsed['extrinsic_xyz'], parsed['extrinsic_R']
        else:
            # 解析失败（已知模型）⇒ 退回"无旋转 + (0,0,0.05)"并 WARN（= 改动前那份配置的值）
            print('[bringup_sim][WARN] 解析器没拿到 T(imu←livox)（%s）⇒ LIO 外参退回 '
                  'extrinsic_T=[0,0,0.05]、extrinsic_R=I（robot11 会少 30° 补偿！）'
                  % (parsed or {}).get('imu_error'), flush=True)
            xyz, rot9 = [0.0, 0.0, 0.05], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        v = xyz if self.__which == 'xyz' else rot9
        return '[%s]' % ', '.join(repr(float(x)) for x in v)

    def describe(self):
        return '%s(%s)' % (type(self).__name__, self.__which)


# =============================================================================
# ★ 2026-10-27（契约 §4.12.5）：**每个 LIO 后端真正读取的外参键名**（唯一一张表）
# -----------------------------------------------------------------------------
#   槽位              键名                                        读取处（file:line）
#   fast_lio          `mapping.extrinsic_T` / `mapping.extrinsic_R`
#                     src/rm_localization/FAST_LIO/src/laserMapping.cpp:834–835（declare）、
#                     870–871（get）
#   point_lio         `mapping.extrinsic_T` / `mapping.extrinsic_R`
#                     src/rm_localization/point_lio/src/parameters.cpp:179–183
#   small_point_lio   `extrinsic_T` / `extrinsic_R`（**顶层**）
#                     src/rm_localization/small_point_lio/src/small_point_lio/parameters.cpp:110–116
#
# 为什么要有这张表：2026-10-27 之前 launch 给三个槽位**都**传顶层 `extrinsic_T/R` ——
# SPL 读顶层（对），fastlio/pointlio 读 `mapping.*`（**死键**）⇒ robot11 的 +30° 静默丢失，
# 运行期 pointlio `mapping.extrinsic_R = I`（measurements §34.3#4 实测）。
# 现在：① 参数字典（`_lio_extrinsic_params`）与 ② 运行期断言
# （`_lio_extrinsic_check_params` → `rm_utils/lio_extrinsic_receipt.py`）**都从这张表派生**
# ⇒ 键名只有一处可改，不会再出现"launch 传了键、节点不读"。
_LIO_EXTRINSIC_KEYS = {
    'fastlio': ('mapping.extrinsic_T', 'mapping.extrinsic_R'),
    'pointlio': ('mapping.extrinsic_T', 'mapping.extrinsic_R'),
    'small_point_lio': ('extrinsic_T', 'extrinsic_R'),
}

#: LIO 节点名（launch 的 `name=` / 源码里的 `Node("...")`）—— 断言节点按名字找参数服务。
_LIO_NODE_NAMES = {
    'fastlio': '/laser_mapping',        # FAST_LIO laserMapping.cpp：`Node("laser_mapping", ...)`
    'pointlio': '/laserMapping',        # 本 launch 给 point_lio 的 `name='laserMapping'`
    'small_point_lio': '/small_point_lio',
}


def _extrinsic_namespace(key):
    """参数键 → 命名空间（`mapping.extrinsic_R` → `mapping`；`extrinsic_R` → ``）。"""
    return key.rsplit('.', 1)[0] if '.' in key else ''


def _lio_extrinsic_params(lio):
    """该槽位的外参参数字典：键名来自 `_LIO_EXTRINSIC_KEYS`，值来自 URDF 解析。

    值本身是 Substitution（惰性求值）⇒ 只有这个槽位真的被选时才展开 xacro。
    """
    key_T, key_R = _LIO_EXTRINSIC_KEYS[lio]
    return {key_T: [_MountExtrinsic('xyz')], key_R: [_MountExtrinsic('rot')]}


def _lio_extrinsic_check_params(lio):
    """该槽位的运行期断言参数（交给 `rm_utils/lio_extrinsic_receipt.py`）。

    `target_node` / `param_prefix` 与 `_lio_extrinsic_params` **同源** ⇒ 断言检查的就是
    launch 写进节点的那两个键；`expected_*` 也是同一份 URDF 解析结果。
    """
    key_T, _ = _LIO_EXTRINSIC_KEYS[lio]
    return {
        'target_node': _LIO_NODE_NAMES[lio],
        'param_prefix': _extrinsic_namespace(key_T),
        'expected_T': [_MountExtrinsic('xyz')],
        'expected_R': [_MountExtrinsic('rot')],
        'timeout_s': 90.0,
        'tol': 1e-9,
        'strict': True,
    }


def _lio_extrinsic_receipt(context):
    """`[lio_extrinsic]` t=0 收据：把"这个 LIO 读哪两个键、期望值、谁断言"写死（一行）。"""
    lio = _Nav2ParamsForSlot._cfg(context, 'lio') or 'fastlio'
    if lio not in _LIO_EXTRINSIC_KEYS:
        return None
    key_T, key_R = _LIO_EXTRINSIC_KEYS[lio]
    parsed = _parsed_livox_mount(context)
    if parsed is not None and parsed.get('extrinsic_R') is not None:
        xyz, rot9 = parsed['extrinsic_xyz'], parsed['extrinsic_R']
        src = parsed.get('source', '?')
    else:
        xyz, rot9 = [0.0, 0.0, 0.05], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        src = '**FALLBACK**（URDF 解析失败，见上面的 WARN）'
    return ('[lio_extrinsic] lio=%s ⇒ 该后端只读 `%s` / `%s`；expected（来源 = %s）'
            'T=[%s]、R=[%s]（行主序）；运行期断言 = rm_utils/lio_extrinsic_receipt.py '
            '（node=%s，读回**生效值**；不一致/死键 ⇒ ERROR + exit 1；契约 §4.12.5）'
            % (lio, key_T, key_R, src,
               ', '.join('%.9f' % float(x) for x in xyz),
               ', '.join('%.9f' % float(x) for x in rot9),
               _LIO_NODE_NAMES[lio]))


class _LioExtrinsicReceipt(Substitution):
    """`LogInfo` 的消息体（惰性求值 ⇒ `--show-args` 不执行、不读参数）。"""

    def perform(self, context):
        return _lio_extrinsic_receipt(context)

    def describe(self):
        return '%s()' % type(self).__name__


class _LioAdapterExtrinsic(Substitution):
    """`lio_tf_adapter` 的 `xyz`/`rpy` = **T(imu_link ← base_link)**，**从这次 spawn 的 xacro 解析**。

    为什么是 imu_link 而不是 livox_frame：三份 LIO 配置的 `extrinsic_T=[0,0,0.05]` 把 livox 原点
    搬到 IMU 系，估计器状态是 **odom←IMU body**；适配器的语义 `t_out = t_lio + q_lio·t_offset`
    要求 t_offset 就是"base_link 原点在 body(=imu_link) 系里的位置" ⇒ 含那 0.05 m。

    ★ 2026-10-21：不再按"三个槽位各一条规则"手写。`_parsed_livox_mount()` 沿
      `base_link → livox_frame → imu_link` 的 **fixed 关节链**合成 T(base←imu)，再取逆得到
      T(imu←base)（xyz = −Rᵀ·p，rpy = Rᵀ 的 rpy）⇒ 云台转动/URDF 改几何都会自动流到这里。
      旧规则表（`_adapter_table`）只在解析失败时回退（已打 WARN）；两边数值在 1b5db4b 的所有
      槽位/挂法上逐位一致（见 docs/frame_contract_v3_measurements.md §10）。
    """

    def __init__(self, which):
        super().__init__()
        self.__which = which

    def perform(self, context):
        robot = _known_robot(context)   # robot 未登记 ⇒ 硬报错（与 _urdf_livox_mount_rpy_deg 同一条）
        parsed = _parsed_livox_mount(context)
        if parsed is not None and parsed.get('adapter_xyz') is not None:
            xyz, rpy = parsed['adapter_xyz'], parsed['adapter_rpy']
        else:
            xyz, rpy = _adapter_table(context)      # 解析失败/imu_link 不在链上 ⇒ 旧表 + WARN 已打过
        _assert_radians('lio_tf_adapter 的 rpy 参数（T(imu<-base_link)）', rpy,
                        'robot=%r which=%s' % (robot, self.__which))
        v = xyz if self.__which == 'xyz' else rpy
        return '[%s]' % ', '.join(repr(float(x)) for x in v)

    def describe(self):
        return '%s(%s)' % (type(self).__name__, self.__which)


class _LioAdapterRobot11(Substitution):
    """把 robot11 的杆臂算成一段 **YAML 列表字符串**（launch_ros 会对 Substitution 的求值结果
    做 yaml.safe_load ⇒ 正好还原成 double 数组；见 launch_ros/utilities/evaluate_parameters.py）。

    ★ 2026-10-21：数值来自 `_parsed_livox_mount()`（展开这次真 spawn 的 xacro → fixed 关节链），
    不再来自倾斜轴规则表（倾角 = 上游 URDF 烘进生成物 xacro 的值）；旧表（`_adapter_table`）
    只在解析失败时回退 + WARN。

    惰性：只有 `robot:=robot11` 且 LIO 不是 none/cartographer/small_point_lio 时才会被求值。
    """

    def __init__(self, which):
        super().__init__()
        self.__which = which

    def perform(self, context):
        parsed = _parsed_livox_mount(context)
        if parsed is not None and parsed.get('adapter_xyz') is not None:
            xyz, rpy = parsed['adapter_xyz'], parsed['adapter_rpy']
        else:
            xyz, rpy = _adapter_table(context)
        _assert_radians('lio_tf_adapter 的 rpy 参数（T(imu<-base_link)）', rpy,
                        'robot=robot11 which=%s' % self.__which)
        v = xyz if self.__which == 'xyz' else rpy
        return '[%s]' % ', '.join(repr(float(x)) for x in v)

    def describe(self):
        return '%s(%s)' % (type(self).__name__, self.__which)


class _Nav2ParamsForSlot(Substitution):
    """nav2 的**公共段**参数文件：按槽位把覆盖层**深合并**进公共段。

    为什么用"运行时合并"而不是多加一份参数文件：`bringup_rm_navigation.py` 只认三个槽
    （params_file / planner / controller），三份都被占满了；而 `src/rm_navigation/**/params`
    是别的任务的目录（本主题禁改）。⇒ 在**我们自己的 launch** 里把覆盖层深合并到公共段之上，
    写成一份临时 YAML 再传下去。

    覆盖层（**全部 opt-in，逐个带命中条件**；一个都不命中 ⇒ 原样返回公共段路径、不写临时文件
    ⇒ 行为与以前逐字节一致）：
      · `robot:=robot11`        → config/nav2_params_sim_robot11_costmap.yaml（代价图几何）
      · `global_obstacle:=scan` → config/nav2_params_sim_global_scan_obstacle.yaml
                                  （全局 2D scan 层的 min_obstacle_height；根因见
                                   docs/frame_contract_v3_measurements.md §15）
      · `scan_band_min != 0.0` + local 2D scan 源
                                → config/nav2_params_sim_local_scan_band.yaml
                                  （**局部** 2D scan 层高度带下界的小负余量；§26。默认 −0.05，
                                   回滚 = `scan_band_min:=0.0` ⇒ 本层不命中，逐字节回到改动前）
    合并是**深**的（只覆盖叶子键）⇒ 公共段里其它一切保持不变；临时文件落在 `$ROS_LOG_DIR`。

    ★ 2026-10-20：覆盖层条目可以是 3 元组 `(tag, path, predicate)`，也可以是 4 元组
      `(tag, path, predicate, bindings)`。`bindings` = `((点分叶子键, Substitution), ...)`：
      在合并之前把该覆盖层里的这个叶子键的值**替换成 launch 参数求值后的数**（YAML 解析，
      不是字符串）⇒ 同一份覆盖文件可以被一个 launch 参数驱动（`scan_band_min`），
      且 `0.0` 时由 predicate 整个跳过（= 逐字节回滚，不留下显式的 0.0 键）。
    """

    def __init__(self, base_path, layers=()):
        """@param layers ((tag, override_path, predicate(context)->bool[, bindings]), ...)；按顺序合并。"""
        super().__init__()
        self.__base = base_path
        norm = []
        for layer in layers:
            if len(layer) == 4:
                tag, path, pred, bindings = layer
            else:
                tag, path, pred = layer
                bindings = ()
            norm.append((tag, path, pred, tuple(bindings)))
        self.__layers = tuple(norm)

    @staticmethod
    def _cfg(context, name, default=''):
        """读 launch 配置（**容错**）：还没声明/取不到 ⇒ 返回 default（不抛 SubstitutionFailure）。"""
        try:
            return str(context.launch_configurations.get(name, default)).strip()
        except Exception:                                             # noqa: BLE001
            return default

    def perform(self, context):
        import tempfile

        import yaml as _yaml
        active = []
        for tag, path, pred, bindings in self.__layers:
            if not pred(context):
                continue
            if not os.path.isfile(path):
                raise RuntimeError('[launch] %s 的 nav2 覆盖层不在：%s' % (tag, path))
            active.append((tag, path, bindings))
        if not active:
            return self.__base
        with open(self.__base) as f:
            base = _yaml.safe_load(f)

        def deep_merge(dst, src):
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dst.get(k), dict):
                    deep_merge(dst[k], v)
                else:
                    dst[k] = v

        def bind(overlay, dotted, value):
            """把覆盖层里的点分叶子键设成 value（覆盖文件只提供**形状/默认值**的兜底）。"""
            node = overlay
            parts = dotted.split('.')
            for p in parts[:-1]:
                node = node.setdefault(p, {})
            node[parts[-1]] = value

        for _tag, path, bindings in active:
            with open(path) as f:
                overlay = _yaml.safe_load(f)
            for dotted, subst in bindings:
                # Substitution 求值后是**字符串**，safe_load 回数值（-0.05 ⇒ float，不是 str）
                bind(overlay, dotted, _yaml.safe_load(subst.perform(context)))
            deep_merge(base, overlay)
        out_dir = os.environ.get('ROS_LOG_DIR') or tempfile.gettempdir()
        out = os.path.join(out_dir, 'nav2_params_sim_%s_merged.yaml' %
                           '_'.join(tag for tag, _p, _b in active))
        with open(out, 'w') as f:
            _yaml.safe_dump(base, f, allow_unicode=True, sort_keys=False)
        return out

    def describe(self):
        return '%s(base=%s, layers=%s)' % (
            type(self).__name__, self.__base, [t for t, _p, _f, _b in self.__layers])


# ★ 2026-10-20：局部 2D scan 高度带下界的默认余量 / 合法域（§26，缺陷 "b"）。
#   默认 −0.05 m：§25 的 hybrid 最低波束端点 +0.007 m，距下界只有 7 mm，而 LIO 的 z 漂移是
#   −0.065…−0.077 m ⇒ 余量放到 57 mm。**只允许下调**（[−1.0, 0.0]）：正数会把下界抬高，
#   让更多波束被判出带（正是本修法要防的静默失效）⇒ 直接硬报错，不静默夹紧。
_SCAN_BAND_MIN_DEFAULT = '-0.05'
_SCAN_BAND_MIN_RANGE = (-1.0, 0.0)


def _scan_band_min_value(context):
    """求值 `scan_band_min`（float，米）；非法值**硬报错**（不静默回退）。"""
    raw = _Nav2ParamsForSlot._cfg(context, 'scan_band_min', _SCAN_BAND_MIN_DEFAULT)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise RuntimeError(
            "[launch] scan_band_min:=%r 不是数（局部 2D scan 高度带下界，单位米；"
            "默认 %s，回滚 scan_band_min:=0.0）" % (raw, _SCAN_BAND_MIN_DEFAULT))
    if not (_SCAN_BAND_MIN_RANGE[0] <= value <= _SCAN_BAND_MIN_RANGE[1]):
        raise RuntimeError(
            "[launch] scan_band_min:=%r 越界：只允许 [%s, %s]（**只许下调**；正数会把下界抬高、"
            "让更多波束出带 ⇒ 拒绝）。回滚 = scan_band_min:=0.0"
            % (raw, _SCAN_BAND_MIN_RANGE[0], _SCAN_BAND_MIN_RANGE[1]))
    return value


def _scan_band_layer_active(context):
    """覆盖层命中条件：**局部 2D scan 源在用** 且 `scan_band_min != 0.0`。

    · `local_obstacle` 的取值表（navigation_launch.py 的 LOCAL_OBSTACLE_LAYER_TABLE）：
      `scan` / `both` / `stvl_both` 才启用 `obstacle_layer`（2D `/scan`）⇒ 只有这三档命中；
      `cloud` / `stvl` 不用 2D scan 源 ⇒ 不命中（合参文件与改动前逐字节相同）。
    · `0.0` = 显式回滚 ⇒ 不命中 ⇒ 覆盖层根本不参与合并（不是"合并一个 0.0"，是**逐字节**回到
      改动前：文件名/内容都与 §25 的 `nav2_params_sim_robot11_merged.yaml` 一致）。
    """
    if _scan_band_min_value(context) == 0.0:
        return False
    return _effective_local_obstacle(context) in ('scan', 'both', 'stvl_both')


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
    """拼出 `xacro <模型> xyz:=… rpy:=… [visual_decimated:=…]` 这条命令（惰性）。

    ★ 2026-10-21：参数列表现由 `_robot_xacro_args()` **唯一真源**生成 —— 与 URDF 解析器
    （`_robot_xacro_mappings`）共用同一份 ⇒ "解析的模型" 与 "spawn 的模型" 在参数上不会分叉。
    对 1b5db4b 的所有槽位/挂法/轴，渲染出的字符串**逐字节不变**（含 robot11 的引号规则）。

    ★ 2026-10-27（契约 §4.15）：hzmirm 不再追加 `turret_yaw_deg` / `turret_pitch_deg`
    （云台固定 0/0）、robot11 不再追加 `livox_mount`（只剩 xacro 默认的 `sensor`）⇒ 命令字符串
    各少一个/两个 token，但 xacro 取默认值 = 同一份输出（measurements §38 的 md5 表）。
    """

    def __init__(self, urdf_dir, xyz, rpy):
        super().__init__()
        self.__slot = _RobotSlot(urdf_dir)
        self.__xyz = xyz
        self.__rpy = rpy

    def perform(self, context):
        return _render_robot_xacro_command(context, self.__slot.perform(context),
                                           self.__xyz, self.__rpy)

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
    """robot:=robot11 的**安装方式**在横幅里的收据（静态文本；参数已删除）。

    ★ 2026-10-08：横幅不能写死"livox_frame 的 rpy=0" —— 当时 `robot11_mount:=urdf` 时它是斜的。
    ★ 2026-10-27（契约 §4.15）：`robot11_mount` **已删除**，安装方式只剩一种（= 2026-10-21 起的
    `sensor` 语义，已烘进 xacro 默认值）⇒ 这里不再按参数分支，只把**当前唯一**的语义如实打出来。
    """

    def perform(self, context):
        return ('  · 雷达安装方式 = **实车约定（2026-10-29 起拆成"物理 + 数据帧"两层）**（契约 §4.12.3）：'
                '`base_link→livox_physical` **rpy = −30°**（物理安装姿态，mesh 挂这里 ⇒ **画出来是斜的**），'
                '射线传感器也挂 `livox_physical` ⇒ 插件发 `/livox/lidar_raw`（点表达在**斜**的物理系）；'
                '`livox_cloud_extrinsic` 复刻驱动器 `extrinsic_parameter`：Rx(+30°) 旋正 → `/livox/lidar` '
                '（`frame_id=livox_frame`，`base_link→livox_frame` **rpy=0**，与实车 TF 一致）；'
                '`livox_frame→imu_link` 仍是倾角（IMU 物理上真的斜 30°）+ LIO `extrinsic_R` 由本 launch '
                '从 URDF 解析成 Rx(+30°)（水平点云 → 斜 IMU 系）、`extrinsic_T` = [0,0,0.05]；'
                '⇒ 下游看到的仍是**水平**点云（地面法向 ≈ +z）⇒ linefit 用槽位那份 '
                '（`sensor_height: 0.2595`、`gravity_aligned_frame: ""`）、p2l 用 `target_frame: base_link`。'
                '`base_link→livox_frame` **rpy=0**（与实车 TF 一致，只有平移 0.204443 m）+ 倾角在 '
                '`livox_frame→imu_link`（IMU 物理上真的斜 30°，加速度计感受得到）+ 插件 '
                '`<tilt_rpy>` 把**射线**旋斜、并把**点**旋进**水平**的 `livox_frame`（= 实车驱动器 '
                '`extrinsic_parameter` 只旋点云的语义）+ LIO 的 `extrinsic_R` 由本 launch 从 URDF '
                '解析成 Rx(+30°)（水平点云 → 斜的 IMU 系）、`extrinsic_T` 仍是 [0,0,0.05]。'
                '⇒ 点云在 `livox_frame` 里**水平**（地面法向 ≈ +z）⇒ linefit 用槽位那份 '
                '（`sensor_height: 0.2595`、`gravity_aligned_frame: ""`）、p2l 用 '
                '`target_frame: base_link`（`/scan` 仍在 base_link）。'
                '⚠️ `odom` 仍是 **IMU 系的原始世界**（`fix_gravity_direction` + 初始 R=I）⇒ '
                '`odom→base_link` 仍是 ~+30°；重力对齐由静态边 `odom→odom_level` 负责（契约 §4.11）')

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

# 惰性加载守卫模块（模块本体已移到 rm_utils：share/rm_utils/scripts/map_asset_guard.py，
# 与 cloud_accumulator、tools/scripts/mapping/map_archive.sh 共用同一份判定）
_MAP_GUARD_CACHE = {}


def _find_source_pkg_dir(share_dir):
    """从 `share/<pkg>/<sub>/` 里挑一个指向别处的软链，反推**仓库源码里的包目录**。

    为什么需要：`colcon build --symlink-install` 下 share 里的每个文件是**单独**的软链
    （目录本身是真的）⇒ 运行期新建的 `map/<name>.posegraph` 不会自动出现在 share 里。
    这里只用它做**兜底**：从 rm_nav_bringup 的 share 反推 src/，再找同级的 rm_utils
    （与 map_asset_guard.canonical_asset_dir() 同一套判断，忘了重编 rm_utils 也能找到）。
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


def _load_map_asset_guard():
    if 'mod' in _MAP_GUARD_CACHE:
        return _MAP_GUARD_CACHE['mod']
    cands = []
    try:
        cands.append(os.path.join(get_package_share_directory('rm_utils'),
                                  'scripts', 'map_asset_guard.py'))
    except PackageNotFoundError:
        pass
    try:
        from ament_index_python.packages import get_package_prefix
        cands.append(os.path.join(get_package_prefix('rm_utils'), 'lib', 'rm_utils',
                                  'map_asset_guard.py'))
    except Exception:                                          # noqa: BLE001 - 只是候选路径
        pass
    # 源码树兜底：rm_nav_bringup 的 share 反推 src/ ⇒ 同级 rm_utils
    try:
        src_pkg = _find_source_pkg_dir(get_package_share_directory('rm_nav_bringup'))
        if src_pkg:
            cands.append(os.path.join(os.path.dirname(src_pkg), 'rm_utils', 'rm_utils',
                                      'map_asset_guard.py'))
    except Exception:                                          # noqa: BLE001
        pass
    path = next((p for p in cands if os.path.isfile(p)), None)
    if path is None:
        raise RuntimeError(
            "[launch] 找不到地图存档守卫模块 map_asset_guard.py（试过：%s）\n"
            '  它现在是 rm_utils 包的一部分，安装方式：\n'
            '      colcon build --symlink-install --packages-select rm_utils\n'
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
    #   robot 留空（默认）= 默认模型 `sentry_robot_sim_rigid.xacro`（2026-10-20 起；它是
    #                       `sentry_robot_sim.xacro` 的**单刚体底盘生成物** —— 差别只有
    #                       轮关节 continuous→fixed 与轮子对地摩擦，几何逐条相等，
    #                       由 tools/scripts/regress/default_make_sim_xacro.py 自证）；
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
    #                 ⚠️ `livox_raise_m` 已从 launch 删除（冻结 0）；若用 scratch 生成器复现 B 方案，
    #                    那份 YAML 的 sensor_height 必须手工同步（0.2595 + raise），本 launch 不替改。
    segmentation_params = _RobotSlotFile(
        'linefit_ground_segmentation_ros',
        ('config', 'segmentation_sim.yaml'),
        error_hint='该文件属于本次新增的 robot 槽位：'
                   'colcon build --symlink-install --packages-select '
                   'linefit_ground_segmentation_ros 后重试；'
                   '或去掉 robot:=<模型> 回到默认模型',
        slot_map={'hzmirm': ('config', 'segmentation_sim_hzmirm.yaml'),
                  'robot11': ('config', 'segmentation_sim_robot11.yaml'),
                  'chassis': ('config', 'segmentation_sim_chassis.yaml')})
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

    ########################## 近地剔除（near_ground）parameters start ################################
    # ★★ 2026-10-09：**按 robot 槽位**选"近地剔除"参数文件（同一套 _RobotSlotFile 机制）。
    #   与自击掩膜（self_mask）是**两件不同的事**：
    #     · self_mask   = "机器人自己的 collision 几何 / 近场死区"里的点（几何固定、与地面无关）；
    #     · near_ground = `dz = z − 局部地面 ≤ 阈值` 的点（判据用**局部地面**判，与车体几何无关）。
    #   为什么必须有这一级（robot11 实测，docs/tilted_lidar_fidelity.md §K）：
    #     `/scan` 是**二维平盘**（LaserScan 没有高度）。nav2 的 `obstacle_layer` 先 `projectLaser`
    #     （z 强行置 0）再 `transformLaserScanToPointCloud` 搬到代价图帧 ⇒ 每条波束在那个帧里的 z
    #     **恒等于"那一帧传感器原点的 z"**（实测：948 条波束的 odom z 全在 [0.020, 0.089]，
    #     `min_obstacle_height 0.0` 一条都不丢）⇒ **`min_obstacle_height` 在 2D 链路上没有逐点
    #     高度可判**。唯一能在"投影成 2D 之前"用上高度的地方就是地面分割节点里的这一级。
    #   不修的后果：近场地面点（水平 0.30–0.50 m）被投成 lethal 格 ⇒ 车心到最近 lethal 0.39 m
    #     （只比本槽位 inscribed 0.3565 大 3~4 cm）⇒ 车半径圆内 ≥99 440/1000、free 仅 26
    #     ⇒ RPP `detected collision ahead` ⇒ **目标被接受但车不走**。
    #   默认文件 = `traversability_near_ground.yaml`（obstacle_near_ground_m: 0.0 = **关**）
    #     ⇒ 默认模型 / 其它槽位**逐字节行为不变**；只有 robot11 读 robot11 那份（0.05）。
    #   参数文件顺序仍是"后者覆盖前者"：本文件放在自击掩膜**之后**（两者键不同，顺序不影响结果，
    #   放在后面只为让"最后一份是槽位专属"这条读法继续成立）。
    near_ground_params = _RobotSlotFile(
        'rm_nav_bringup',
        ('config', 'traversability_near_ground.yaml'),
        error_hint='该文件属于 2026-10-09 的近地剔除：'
                   '`colcon build --symlink-install --packages-select rm_nav_bringup` 后重试；'
                   '或删掉 launch 里这一路参数（回到"没有这一级"的行为）',
        slot_map={'robot11': ('config', 'traversability_near_ground_robot11.yaml')})
    ########################## 近地剔除（near_ground）parameters end ##################################

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
    # ★ 2026-10-07 Phase 3：公共段按槽位走一遍**深合并**（一个覆盖层都不命中时原样返回同一份
    #   base ⇒ 默认槽位逐字节不变）。
    #   · robot:=robot11            → 覆盖层只含代价图几何键（足印内切 0.300 / 外接 0.3565）——
    #     rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml
    #     （文件头与 docs/robot_models.md §11.6/§11.9）。
    #   · global_obstacle:=scan     → 覆盖层只含**全局 2D scan 层的 min_obstacle_height**（−1.0）——
    #     rm_nav_bringup/config/nav2_params_sim_global_scan_obstacle.yaml。
    #     为什么必须有这一层：扫描面在 map 帧里的 z ≈ −0.18 m，被 nav2 的 z 闸**静默**全丢
    #     ⇒ 全局图恒等于 static+inflation（根因/实测见 docs/frame_contract_v3_measurements.md §15）。
    #     默认 `global_obstacle:=stvl` 不命中 ⇒ 参数文件逐字节不变。
    #   · scan_band_min != 0.0 + 局部 2D scan 源
    #                               → 覆盖层只含**局部 2D scan 层的两个 min_obstacle_height**（默认
    #     −0.05，值由 launch 参数绑定）—— rm_nav_bringup/config/nav2_params_sim_local_scan_band.yaml。
    #     为什么必须有这一层：§25 的 hybrid 最低波束端点只有 +0.007 m（LIO z 漂移 −0.065…−0.077），
    #     7 mm 余量太薄；−0.05 给 57 mm。回滚 `scan_band_min:=0.0` ⇒ 不命中、逐字节回到改动前。
    #     **只写 local_costmap**：STVL 的 3D 带与全局 −1.0 覆盖一个字节都不动（§26）。
    nav2_base_params_for_slot = _Nav2ParamsForSlot(
        os.path.join(nav2_params_dir, 'nav2_params_sim_base.yaml'),
        layers=(
            ('robot11',
             os.path.join(rm_nav_bringup_dir, 'config', 'nav2_params_sim_robot11_costmap.yaml'),
             lambda ctx: _Nav2ParamsForSlot._cfg(ctx, 'robot') == 'robot11'),
            ('global_scan',
             os.path.join(rm_nav_bringup_dir, 'config',
                          'nav2_params_sim_global_scan_obstacle.yaml'),
             lambda ctx: _Nav2ParamsForSlot._cfg(ctx, 'global_obstacle') == 'scan'),
            ('local_scan_band',
             os.path.join(rm_nav_bringup_dir, 'config',
                          'nav2_params_sim_local_scan_band.yaml'),
             _scan_band_layer_active,
             # 覆盖文件里的 −0.05 只是默认值/文档；真值来自 launch 参数（`0.0` 时整层不命中）。
             (('local_costmap.local_costmap.ros__parameters.obstacle_layer.min_obstacle_height',
               LaunchConfiguration('scan_band_min')),
              ('local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.min_obstacle_height',
               LaunchConfiguration('scan_band_min')))),
            # ★ 2026-10-21（契约 §4.11/§4.14）：局部帧由 `gravity` 派生（tf ⇒ odom_level；
            #   base|lio|raw ⇒ odom）。只改**两个叶子键**：local_costmap.global_frame 与
            #   behavior_server.global_frame（后者吃的就是这张局部代价图）。全局仍是 map、
            #   stvl 的全局 profile 与 global_obstacle:=scan 的 −1.0 覆盖层**一个字节都不动**。
            #   非 tf 档 ⇒ 本层 predicate 为假 ⇒ 合并文件与改动前逐字节相同。
            ('level_frame',
             os.path.join(rm_nav_bringup_dir, 'config', 'nav2_params_sim_level_frame.yaml'),
             lambda ctx: _level_frame(ctx) == 'odom_level',
             (('local_costmap.local_costmap.ros__parameters.global_frame', _LevelFrame()),
              ('behavior_server.ros__parameters.global_frame', _LevelFrame()))),
        ))
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
        "{'RMUC': 0.0, 'RMUL': 0.0, 'RMUL2026': 0.0, 'RMUC2026': 0.0,"
        " 'RMUC2027': 0.0, 'RMUL2027': 0.0}['",
        LaunchConfiguration('world'), "']"])
    amcl_init_y = PythonExpression([
        "{'RMUC': 0.0, 'RMUL': 0.0, 'RMUL2026': 0.0, 'RMUC2026': 0.0,"
        " 'RMUC2027': 0.0, 'RMUL2027': 0.0}['",
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
    # ★ 2026-10-28（契约 §4.16）：global_relocalizer:=scan_context 的参数（同 `_PackageShareFile`
    #   的惰性原则 —— 不选该槽位就永不 perform；缺包只挡这一条）。
    sc_relocalizer_params = _PackageShareFile(
        'scan_context_relocalizer', 'config', 'scan_context_relocalizer_sim.yaml')
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

    # ★ 2026-10-28（契约 §4.16）：`global_relocalizer:=scan_context` 的节点定义。
    #   条件 = nav + 选了 scan_context + lio!=cartographer（cartographer 全包形态自己发 map→odom，
    #   不消费 /initialpose；它继续由 localization 槽的语义管）。默认 `none` ⇒ condition 恒假
    #   ⇒ 节点集/话题集/时序与改造前**逐字节相同**。
    #   参数：DB 路径按 world 惰性解析（缺文件响亮失败），其余走包内 YAML；
    #   `auto_start_delay_sec` 默认 8 s，等 LIO/TF（与 localization 组 4 s 起、gicp 7 s 起的既有
    #   时序同量级）。
    sc_relocalizer_node = Node(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'nav' and '",
            LaunchConfiguration('global_relocalizer'), "' == 'scan_context' and '",
            LaunchConfiguration('lio'), "' != 'cartographer'"])),
        package='scan_context_relocalizer',
        executable='scan_context_relocalizer_node',
        name='scan_context_relocalizer',
        output='screen',
        parameters=[
            sc_relocalizer_params,
            {'use_sim_time': use_sim_time,
             'db_path': _WorldScDbFile()},
        ],
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='RMUL2026',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    # ★ 2026-10-18：**仿真后端**槽位（`classic` | `gz`）。**纯增量**：
    #   · 不传（默认 `classic`）⇒ 本文件里每一个 Action、每一条命令行、每一处时序
    #     **逐字节不变**（`start_rm_simulation` 的 condition 恒为真 = "加了门但门一直开"）；
    #   · `gz` ⇒ 只把「起仿真世界」这一件事换成 Gazebo Fortress（ignition-gazebo6）：
    #     `rm_livox_gz_sim/launch/gz_sim.launch.py` 起 ign gazebo + /clock 桥 + TF，
    #     感知 / LIO / nav2 全部**原样**复用本文件后面的节点（契约对照见
    #     docs/fortress_sim_backend.md）。
    #   ⚠️ `gz` 档的两个硬要求（缺了**明确报错**，不静默跑错）：
    #     ① `robot:=robot11` —— 只有这个槽位在 gz 侧有 Fortress 化的模型生成路径；
    #     ② 雷达安装方式 = 唯一的 `sensor` 语义（倾角必须在 URDF 里，TF 才与物理一致）。gz 侧
    #        生成的 `<link><pose>` 也真的带 roll −0.5236 ⇒ 点云数字在**斜的传感器系**里。
    #        ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已删除，**不需要再传**；gz_sim.launch.py
    #        自己的同名参数默认 `sensor`（本文件不再转发它）。
    declare_sim_backend_cmd = DeclareLaunchArgument(
        'sim_backend',
        default_value='classic',
        choices=['classic', 'gz'],
        description='仿真后端：classic（默认；Gazebo Classic 11，本文件现行行为逐字节不变）'
                    '| gz（opt-in；Gazebo Fortress = ignition-gazebo6。需先跑 '
                    'tools/scripts/gzsim/fetch_and_patch_mid360.sh，且必须 '
                    'robot:=robot11；雷达安装方式自 2026-10-27 起固定为 sensor，不再有 '
                    'robot11_mount 参数）')

    # ★ 2026-10-07：机器人模型槽位（**opt-in**，默认值 = 本仓现行模型 ⇒ 现有行为不变）。
    declare_robot_cmd = DeclareLaunchArgument(
        'robot',
        default_value='',
        description="机器人模型槽位（留空 = 本仓默认模型）："
                    "'' → urdf/sentry_robot_sim_rigid.xacro（**单刚体底盘**；几何与 "
                    "urdf/sentry_robot_sim.xacro 逐条相等（轮 r=0.06、base_link 离地 0.06、"
                    "雷达 base_link+0.12,0,0.175 离地 0.226），差别只有①轮关节 continuous→fixed "
                    "②轮子对地摩擦 0.02 ⇒ planar_move 的角速度通道 4.3%→95.4%；"
                    "生成器 tools/scripts/regress/default_make_sim_xacro.py，"
                    "回退 --chassis articulated 或把 _ROBOT_SLOTS[''] 指回源文件）；"
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
                    "默认模型的改动（实测数字/影响面）见 docs/frame_contract_v3_measurements.md §20，"
                    "范围/数值对照见 docs/robot_models.md",
        choices=['', 'hzmirm', 'robot11', 'chassis', 'chassis_boxvis'])

    # ★ 2026-10-21：robot:=robot11 的安装倾角与抬高都不再有 launch 开关：
    #   `livox_tilt_axis` / `livox_raise_m` 已删除（§28），`livox_tilt_rpy` 也不再由本 launch 传
    #   （§31 收口）：倾角的**唯一来源 = 上游 URDF 的 `body_to_livox.rpy`** → 生成器烘进生成物
    #   xacro 的 arg 默认值。要换倾角只能改上游 URDF + 重跑生成器；scratch 实验用
    #   `robot11_make_sim_xacro.py --livox-tilt-rpy "r p y"`（弧度，见契约 §4.12）。
    #   抬高冻结 0（上游几何）；xacro 仍保留 `livox_raise_m` arg 且默认 0 ⇒ 生成物逐字节不变。
    #   ⚠️ 这两个值都是"生成模型的输入"，不是运行时真值：安装 rpy/xyz 由 URDF 解析
    #   （`_parsed_livox_mount()`）唯一定义，所以删除开关不会改变默认路径。

    # ★ 2026-10-27（契约 §4.15）：`robot11_mount` **已删除**（不再声明；`--show-args` 里没有它）。
    #   它的唯一取值 `sensor` 的语义已烘进生成物 xacro 的 `<xacro:arg ... default="sensor"/>`，
    #   所以默认路径的 `robot_description` 逐字节不变（measurements §38）。
    #   ⚠️ 传这个参数**不会**被 ROS 2 当成错误（Humble 对未知 `name:=value` 静默接受，§28.4）；
    #   本 launch 用 `_check_removed_launch_args()` 在 t=0 **硬报错**，消息指向本节。
    #   历史三档（plugin/urdf/sensor）与取证见 docs/tilted_lidar_fidelity.md §J / 契约 §4.15。

    # ★★ 2026-10-21（契约 v3 §4.14）：**重力/水平帧的唯一用户开关** `gravity`。
    #   旧参数组（level_frame / gravity_align_mode / gravity_align_z_origin / z_origin_source）
    #   与两个初始化参数（lio_init_settle_s / lio_init_map_extrinsic）**只作为硬报错别名保留**：
    #   传非默认值 ⇒ t=0 停（`_check_gravity_contract`），消息指向 gravity。
    #   声明顺序：gravity 必须最先（`_level_frame` / `_effective_gravity_align_mode` 都读它）。
    declare_gravity_cmd = DeclareLaunchArgument(
        'gravity',
        default_value=_GRAVITY_DEFAULT,
        choices=list(_GRAVITY_MODES),
        description='重力/水平帧的唯一开关（契约 v3 §4.14）。'
                    'tf（默认）= odom 保持**原始**（IMU）世界、E1 关；局部代价图/静态边帧 = '
                    'odom_level（唯一发布者 odom_level_tf，latched；值从 URDF 的安装 rpy + 离地'
                    '高度 h 算，z=0 = 地面）；障碍层按 local_obstacle（默认 2D scan / stvl）。'
                    'base = odom 保持原始、**没有 odom_level 帧**、局部代价图 = odom；高度语义放在 '
                    '**base_link**（p2l target_frame=base_link、linefit gravity_aligned_frame='
                    'base_link）；局部障碍层强制**逐点**（cloud|stvl；显式 local_obstacle:=scan ⇒ '
                    '硬报错，因为 2D scan 平盘在斜 odom 里没有逐点高度 = §25 的 43.7% 半盲）。'
                    'lio = E1 hybrid 把 odom 摆正（**只对 lio:=small_point_lio**，其它 LIO 硬报错）；'
                    '局部代价图 = odom；z 原点 = 地面；障碍层按 local_obstacle。'
                    'raw = E1 关、局部代价图 = odom、障碍层照旧（2D scan 允许 = 复现 §25 的 off 半盲臂）。'
                    '旧名字（gravity_align_mode / gravity_align_z_origin / level_frame / '
                    'z_origin_source / lio_init_settle_s / lio_init_map_extrinsic）传非默认值 ⇒ '
                    't=0 硬报错。')

    # ---- 旧参数：硬报错别名（保留声明只为让 --show-args 还能看到"它被 gravity 取代"）----
    declare_level_frame_cmd = DeclareLaunchArgument(
        'level_frame',
        default_value='odom_level',
        choices=['odom_level', 'odom'],
        description='[已删除 / 硬报错别名] 由 gravity 一次性决定：'
                    'tf ⇒ odom_level；base|lio|raw ⇒ odom。传非默认值（odom）⇒ t=0 停。'
                    '见 gravity 的 description / 契约 §4.14')

    declare_gravity_align_mode_cmd = DeclareLaunchArgument(
        'gravity_align_mode',
        default_value='auto',
        choices=list(_GRAVITY_ALIGN_MODES),
        description='[已删除 / 硬报错别名] E1 由 gravity 一次性决定：'
                    'lio ⇒ hybrid（只对 lio:=small_point_lio）；tf|base|raw ⇒ off。'
                    '传非默认值（off|urdf|imu|hybrid）⇒ t=0 停。见 gravity / 契约 §4.14')

    declare_gravity_align_z_origin_cmd = DeclareLaunchArgument(
        'gravity_align_z_origin',
        default_value='ground',
        choices=['sensor', 'ground'],
        description='[已删除 / 硬报错别名] z 原点由 gravity 一次性决定：'
                    'tf ⇒ ground + URDF 的 h；lio ⇒ E1b ground；base/raw ⇒ n/a。'
                    '传非默认值（sensor）⇒ t=0 停。见 gravity / 契约 §4.14')

    declare_lio_init_settle_cmd = DeclareLaunchArgument(
        'lio_init_settle_s',
        default_value='1.0',
        description='[已删除 / 硬报错别名] 初始化**不再等落定**（17e468d 起出生即落定；'
                    '落定门行为已硬编码为"不等"）。传非默认值 ⇒ t=0 停。见 gravity / 契约 §4.14')

    declare_lio_init_map_extrinsic_cmd = DeclareLaunchArgument(
        'lio_init_map_extrinsic',
        default_value='True',
        choices=['True', 'False'],
        description='[已删除 / 硬报错别名] 初始化地图**一律**用 R_IL·p + t_IL'
                    '（与逐点更新同口径；R_IL=I 的快路径与一般式逐位相同）。'
                    '传非默认值（False）⇒ t=0 停。见 gravity / 契约 §4.14')

    declare_z_origin_source_cmd = DeclareLaunchArgument(
        'z_origin_source',
        default_value='urdf',
        choices=['urdf', 'measured'],
        description='[已删除 / 硬报错别名] 静态边的 z 原点来源固定 = URDF 的 h_imu'
                    '（确定性、可跨跑比较）。传非默认值（measured）⇒ t=0 停。'
                    '见 gravity / 契约 §4.14')

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

    # ★ 2026-10-27（契约 §4.15）：`turret_yaw_deg` / `turret_pitch_deg` **已删除**（不再声明；
    #   `--show-args` 里没有它们）。为什么它们曾经存在：`robot:=hzmirm` 上游 URDF 里两个云台关节
    #   都是 **fixed**，本仓**没有 gimbal 控制器** ⇒ 它们从来不是"控制量"，只是 spawn 时的**静态
    #   姿态**（默认 0/0 = 与上游数值等价）；pitch≠0 还会破坏雷达重力对齐。现在云台**固定**
    #   在 yaw=0 / pitch=0（生成物逐字节不变、`h = 0.8 m` 不变，measurements §38）。
    #   ⚠️ 传这两个参数会在 t=0 被 `_check_removed_launch_args()` **硬报错**（不静默忽略）。

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

    # ★ 2026-10-28（契约 §4.16）：`global_relocalizer` 槽 —— **全局重定位器**（可选）。
    #   默认 `none` = 当前行为（不新增节点/话题）；`scan_context` = 起 scan_context_relocalizer。
    #   它与 `localization` **正交**：后者持有 map→odom，前者只发 `/initialpose` 假设（I3/I4）。
    declare_global_relocalizer_cmd = DeclareLaunchArgument(
        'global_relocalizer',
        default_value='none',
        choices=['none', 'scan_context'],
        description='全局重定位器槽（与 localization 正交；只发 /initialpose + ~/health，绝不发 TF）: '
                    'none（默认，当前行为不变）| '
                    'scan_context（Scan Context keyframe DB 检索 —— 需预先建 '
                    'PCD/<world>_sc_db.npz；`mode:=nav` 时生效，缺 DB 会响亮失败）。'
                    '为 localization:=gicp|small_gicp|icp|amcl 等提供"没有先验"时的粗位姿。')

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

    # ★ 2026-10-20：局部 2D scan 高度带下界的**小负余量**（缺陷 "b"；§26）。
    #   现状（§25）：hybrid 默认档 local `/scan` 771/771 在带内，但**最低端点只有 +0.007 m**，
    #   而 LIO z 漂移 −0.065…−0.077 ⇒ 只剩 7 mm，一次小扰动就会让波束整批掉出 `[0,2)`、nav2
    #   静默丢点、默认档退回 `off` 的 kept-y 单侧"半盲"状态。默认 −0.05 ⇒ 余量 57 mm。
    #   只作用于**局部 2D scan**（`obstacle_layer` 的两道 min 闸）；STVL 的带、全局 `−1.0`
    #   覆盖、`max 2.0` 都不动。回滚 = `scan_band_min:=0.0`（覆盖层不命中 ⇒ 参数文件逐字节回到
    #   改动前）。取值域 [−1.0, 0.0]：正数会把下界抬高 ⇒ 硬报错。
    declare_scan_band_min_cmd = DeclareLaunchArgument(
        'scan_band_min',
        default_value=_SCAN_BAND_MIN_DEFAULT,
        description='局部 2D scan 高度带**下界**（米；默认 -0.05，回滚 0.0）。只改 '
                    '`local_costmap.obstacle_layer[.scan].min_obstacle_height` 两处（nav2 两道 z 闸），'
                    '`max_obstacle_height` 仍是 2.0；STVL 的 3D 带与全局 `global_obstacle:=scan` 的 '
                    '−1.0 覆盖都不受影响。0.0 = 覆盖层不命中 ⇒ 与改动前逐字节相同。'
                    '背景：§25 hybrid 最低波束端点 +0.007 m vs LIO z 漂移 −0.065…−0.077 m（余量 7 mm）')

    # ★ 2026-10-21：`band_guard` —— nav2 高度带**静默丢波束**的只读诊断（opt-in）。
    #   默认 False = **节点集/参数/时序与改造前完全一致**（这个 Node 的 condition 恒假，
    #   launch 不会起它；只有多一条 launch 参数声明）。见
    #   docs/frame_contract_v3_measurements.md §17。
    declare_band_guard_cmd = DeclareLaunchArgument(
        'band_guard',
        default_value='False',
        choices=['True', 'False'],
        description='False（默认，行为不变）= 不起 band_guard；True = 起只读诊断节点 '
                    '`rm_utils/band_guard.py`：订阅 /scan + TF，回读两张 costmap 的**生效**'
                    '高度带参数，若 in-band==0（ERROR）/ <10%（WARN）/ kept-y 单侧（WARN）就'
                    '打日志 + 发 /band_guard/status(JSON)。**不改任何数据/控制路径**。'
                    '只在 mode:=nav|slam_nav 生效。')

    declare_mapper_cmd = DeclareLaunchArgument(
        'mapper',
        default_value='slam_toolbox',
        choices=['cartographer', 'slam_toolbox'],
        description='Choose 2D mapping backend (only mode:=mapping): slam_toolbox | cartographer')

    # ★ 2026-10-29（meas §46.3）：slam_toolbox 的**节点密度**覆盖（只影响 mode:=mapping 的
    #   slam_toolbox；<=0 = 用 `_MAPPER_PARAMS_BY_WORLD` 的 per-world 槽）。默认 -1 ⇒
    #   2027 = 槽值、2026 = 参数文件原值（0.2/0.2）⇒ 默认路径**逐字节不变**。
    #   ★ 2026-10-30（meas §48）：2027 槽值从 0.6/0.5 换成 A/B 赢家的 0.5/0.3（仍是 -1 默认）；
    #     下面两条 description 里的 "0.6/0.5" 是**故意冻结**的历史文本 —— 改了会让
    #     `ros2 launch ... --show-args` 不再逐字节相同；实际生效值以槽表 / §48 为准。
    #   给 0.2/0.2 = 复现 §45.2 的坏 map→odom（A/B 对照用）。
    declare_slam_min_travel_distance_cmd = DeclareLaunchArgument(
        'slam_min_travel_distance',
        default_value='-1.0',
        description='slam_toolbox minimum_travel_distance 覆盖（m；<=0 = 用 per-world 槽：'
                    '2027 = 0.6，2026 = 参数文件的 0.2）。只影响 mode:=mapping 的 slam_toolbox。')
    declare_slam_min_travel_heading_cmd = DeclareLaunchArgument(
        'slam_min_travel_heading',
        default_value='-1.0',
        description='slam_toolbox minimum_travel_heading 覆盖（rad；<=0 = 用 per-world 槽：'
                    '2027 = 0.5，2026 = 参数文件的 0.2）。只影响 mode:=mapping 的 slam_toolbox。')

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

    # ★ 2026-10-21（§24）：累加器的**累积坐标系**（`map` | `odom`）。默认 `map` = 与改造前逐字节相同。
    #   为什么需要它（实测，见 docs/frame_contract_v3_measurements.md §23.9#1 / §24.5）：
    #   累加器按 `TF(target_frame ← livox_frame)` 的**当前值**把每帧点搬进 target_frame，
    #   而 `slam_toolbox` 会在整跑过程中不断修正/跳变 `map→odom` ⇒ **已经积进去的点不会跟着动**
    #   ⇒ 同一面墙被画成多条弧带（"鬼影"，实测墙面残差散布 0.2–0.4 m）。
    #   `odom` 档：LIO 的 `odom→livox_frame` 是**平滑**的（没有跳变）⇒ 云内部刚性、无鬼影；
    #   落盘后由调用方用**一次** `map←odom` 刚体变换把 PCD 搬进 `map`（sidecar 的 `frames`
    #   仍然写 `map`，因为最终资产确实在 map 系）。2D 栅格那一侧完全不受影响（它自己闭环）。
    declare_cloud_accumulator_target_frame_cmd = DeclareLaunchArgument(
        'cloud_accumulator_target_frame',
        default_value='map',
        description='cloud_accumulator 的累积坐标系：map（默认 = 历史行为）| odom。'
                    'map 档会被 slam_toolbox 的 map→odom 修正撕裂出鬼影（§23.9#1）；'
                    'odom 档把累积放进平滑的 LIO 世界系、落盘后再用一次 map←odom 搬进 map。'
                    '只影响 3D 先验，不影响 /map、map→odom、/segmentation 的单一发布者契约')

    # ★ 2026-10-06（事故当天追加）：续建后的**一次性**一致性检查（详见 docs/continue_mapping.md §8）
    #   动机：场地隔离守卫只管"world/spawn 对不对"，管不了"图本身歪了"——
    #   当天用户续建的 map/RMUC2026.posegraph 里已经含有一段 LIO 退化（ATE max 1.15 m），
    #   加载进来的图与真实场地不一致 ⇒ 建出来的图/定位"飘"。
    #   做法：续建时额外起一个**一次性**小节点（rm_utils/map_resume_check.py），
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
            # ★ 2026-10-21（契约 §4.13.5）：出生高度 = 该模型落定时的 base_link 高度 + 1 mm
            #   （robot11 0.103499 / hzmirm 0.001 / 默认 0.061；只有 floor z=0 的 RMUC2026/RMUL2026
            #   才传，别处返回 '' = 用 world 配置的 z，逐字节回到改造前）。动机见 `_SpawnZForRobot`。
            'spawn_z': _SpawnZForRobot(),
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
            'rviz': 'False'}.items(),
        # ★ 2026-10-18：sim_backend 的门。默认 classic ⇒ 条件恒真 ⇒ 这个 IncludeLaunch-
        #   Description 的参数/命令行/时序与改造前逐字节相同（证据：
        #   `tools/scripts/gzsim/launch_increment_diff.py` 对比 HEAD 前后 --show-args 与
        #   整棵 Action 树的转储）。选 gz 时它整块不执行，由下面的 start_gz_simulation 接手。
        condition=LaunchConfigurationEquals('sim_backend', 'classic'),
    )

    # ★ 2026-10-18：`sim_backend:=gz` 的仿真世界入口（**只在 gz 档执行**）。
    #   它只负责"把世界起起来 + /clock + TF"，本文件后面的感知/LIO/nav2 节点全部原样复用。
    start_gz_simulation = IncludeLaunchDescription(
        # 用 _PackageShareFile 做**惰性**路径解析：没装 rm_livox_gz_sim 时，只要不选
        # sim_backend:=gz，这个路径就永远不会被 perform ⇒ 默认路径零影响（--show-args 也是）。
        PythonLaunchDescriptionSource(
            [_PackageShareFile('rm_livox_gz_sim', 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'world': world,
            'robot': LaunchConfiguration('robot'),
            'robot_description': robot_description,
            # ★ 2026-10-27（契约 §4.15）：不再传 `robot11_mount` —— 本 launch 已没有这个参数，
            #   gz_sim.launch.py 的**同名声明也已删除**（安装方式固定 = xacro 默认 `sensor`）
            #   ⇒ 启动参数与删除前传 `sensor` 逐字节相同（gz 生成物 md5 见 measurements §38.2e）。
            'gui': gui,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=LaunchConfigurationEquals('sim_backend', 'gz'),
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

    # ★ 2026-10-21（契约 §4.12，**按实车改约定**）：`robot11_mount:=sensor` 的
    #   `gravity_aligned_frame: base_link` **覆盖文件已删除** —— 点云现在本来就是**水平**的
    #   （`base_link→livox_frame` rpy=0、插件把点旋进这个帧），linefit 槽位那份
    #   `segmentation_sim_robot11.yaml`（`sensor_height: 0.2595` + `gravity_aligned_frame: ""`）
    #   的语义重新成立 ⇒ 与原 'plugin' 档走同一份参数文件。
    # ★★ 2026-10-29（契约 §4.12.3，**实车分解**的最后一步）：驱动器那样的外参。
    #   链路：`livox_physical`（物理斜 30°，mesh + 射线传感器都挂它）→ 插件发
    #   `/livox/lidar_raw`（点表达在**斜的**物理系）→ 本节点旋 Rx(+30°)（= 安装倾角的逆）
    #   → `/livox/lidar` + `frame_id=livox_frame`（**水平**数据帧）。
    #   ⇒ 下游（linefit / p2l / LIO）看到的仍是水平的 `/livox/lidar`，数据链一字未改。
    #   旋转默认**从 TF 读** `livox_frame←livox_physical`（唯一真源 = URDF 的 body_to_livox_physical）；
    #   其它机器人模型没有 `/livox/lidar_raw`（插件仍直接发 `/livox/lidar`）⇒ 本节点是空转，无副作用。
    bringup_livox_cloud_extrinsic_node = Node(
        package='cloud_accumulator',
        executable='livox_cloud_extrinsic',
        name='livox_cloud_extrinsic',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/livox/lidar_raw',
            'output_topic': '/livox/lidar',
            'frame_id': 'livox_frame',
            'source_frame': 'livox_physical',
            'extrinsic_rpy': [0.523598775598293, 0.0, 0.0],
            'use_tf': True,
        }],
    )

    def _linefit_node(params, condition):
        return Node(
            package='linefit_ground_segmentation_ros',
            executable='ground_segmentation_node',
            output='screen',
            # ★ 2026-09-23 修复：原来漏了 use_sim_time ⇒ 这个节点跑在**墙钟**上，而全链路（plugin/scan/
            #   costmap/AMCL/tf）都是仿真钟。它的输出戳虽然抄自输入（所以看起来还好），但任何依赖
            #   "本节点时钟"的逻辑（tf2 Buffer 的缓存窗口、超时判定）都会用错时间轴。
            parameters=params,
            condition=condition,
        )

    #   ↑ 与 2026-10-07 起的行为**逐个参数文件相同**。
    # ★ 2026-10-06：地面分割槽位 ground（默认 linefit ⇒ 本节点照旧启动，行为不变）。
    #   ⚠️ 历史（2026-10-09→2026-10-21）：曾有一个"`robot11_mount:=sensor` 专属、多一层
    #   `gravity_aligned_frame: base_link` 覆盖文件"的互斥 linefit；该覆盖文件已随 §4.12 删除，
    #   本文件现在只有 `linefit` 与 `gravity:=base` 两份互斥节点（见下）。
    # ★★ 2026-10-21（契约 §4.14）：`gravity:=base` 的**第二份互斥 linefit**（同一个节点名/可执行
    #   文件，只多一个 `gravity_aligned_frame: base_link` 覆盖键）。
    #   为什么 base 档要它：base 档把"高度语义"放在 base_link —— linefit 的 sensor_height 是
    #   "雷达在传感器系下离地高度"，只有先把点旋进 base_link（= 重力对齐的车体系）这个数才成立；
    #   默认档（tf）点云本来水平 ⇒ 保持 `gravity_aligned_frame: ""`（参数表与今天逐字段相同）。
    _linefit_common_params = [segmentation_params, *traversability_params_list, self_mask_params,
                              near_ground_params, {'use_sim_time': use_sim_time}]
    bringup_linefit_ground_segmentation_node = _linefit_node(
        list(_linefit_common_params),
        IfCondition(PythonExpression([
            "'", LaunchConfiguration('ground'), "' == 'linefit' and '",
            _Gravity(), "' != 'base'"])))
    bringup_linefit_ground_segmentation_node_base = _linefit_node(
        [*_linefit_common_params, {'gravity_aligned_frame': 'base_link'}],
        IfCondition(PythonExpression([
            "'", LaunchConfiguration('ground'), "' == 'linefit' and '",
            _Gravity(), "' == 'base'"])))

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
            # ★ 2026-10-30：按 robot 槽位选 patchwork 参数文件（linefit 同款 _RobotSlotFile）——
            #   默认模型 sensor_height 0.226，起伏底盘 0.188（base_link 离地 0.098 + 雷达 0.09）。
            _RobotSlotFile(
                'patchwork_ground_segmentation',
                ('config', 'ground_segmentation_sim.yaml'),
                error_hint='该文件属于 patchwork 地面分割槽位：'
                           '`colcon build --symlink-install --packages-select '
                           'patchwork_ground_segmentation` 后重试；'
                           '或去掉 ground:=patchwork 回到 linefit',
                slot_map={'chassis': ('config', 'ground_segmentation_sim_chassis.yaml')}),
            *traversability_params_list,
            self_mask_params,
            near_ground_params,
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
    #   ★ 2026-10-09：`robot11_mount:=sensor` 这一档不成立（点云真的在斜的传感器系里）⇒
    #   只有这一档递进 `laserscan_params_sensor_frame.yaml`（**只含 `target_frame: base_link`**），
    #   让 `/scan` 回到"重力对齐的水平面里的一圈"、`min/max_heights` 恢复"离地高度"语义。
    #   代价与本仓 2026-09-23 的取舍（TF/MessageFilter 故障面）登记在 docs/tilted_lidar_fidelity.md §J。
    # ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已删除 ⇒ **只保留一份** p2l 节点。删除前
    #   唯一合法值是 `sensor`，所以 `target_frame: base_link` 那份增量覆盖**一直是实际生效的**
    #   （`!= 'sensor'` 的门在参数删除前就恒假）⇒ 现在去掉门，起进程的节点集合与删除前
    #   逐字段相同（measurements §38 的 visited diff）。
    def _p2l_node(extra_params):
        return Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in',  ['/segmentation/obstacle']),
                        ('scan',  ['/scan'])],
            # 参数已回归 pointcloud_to_laserscan 包 config/（R1）
            # ★ 2026-09-23 修复：同上，原来漏了 use_sim_time。p2l 内部用
            #   `tf2_ros::Buffer(this->get_clock())` 建 TF 缓存，节点时钟是墙钟时缓存窗口走的是墙钟，
            #   与消息里的仿真戳不在同一条时间轴上（本配置 target_frame==点云 frame，过滤器短路才没炸；
            #   一旦改 target_frame 就会立刻表现为"整条 Talker 被丢光"）。
            parameters=[os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                                     'config', 'laserscan_params.yaml'),
                        *extra_params,
                        {'use_sim_time': use_sim_time}],
            name='pointcloud_to_laserscan')

    # 「点云在斜系」的唯一安装约定：`target_frame: base_link`（只含这一个键的增量覆盖文件）。
    bringup_pointcloud_to_laserscan_node = _p2l_node(
        [_PackageShareFile('pointcloud_to_laserscan', 'config',
                           'laserscan_params_sensor_frame.yaml')])

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
                    # ★ 2026-10-21（契约 §4.12）：**LIO 外参从 URDF 解析**（点云水平、IMU 斜装）
                    #   ⇒ robot11 的 extrinsic_R = Rx(+30°)、extrinsic_T = [0,0,0.05]（不变）；
                    #   非斜装槽位解析出来就是单位阵/同一平移 ⇒ 与配置里的默认值逐位相同。
                    # ★ 2026-10-27（契约 §4.12.5）：键名**由 `_LIO_EXTRINSIC_KEYS` 决定**。
                    #   FAST_LIO 只读 `mapping.extrinsic_T/R`（laserMapping.cpp:834–835/870–871）；
                    #   原来这里传的是顶层 `extrinsic_T/R` = **死键**（节点一个都没读到）。
                    _lio_extrinsic_params('fastlio'),
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
            # ★ 2026-10-27（契约 §4.12.5）：**运行期断言** —— 读回这个节点**生效的**
            #   `mapping.extrinsic_T/R`，与上面同一份 URDF 解析值比较；不一致/死键 ⇒
            #   ERROR + exit 1（以后键名再写错，不会再"照跑但 R=I"）。
            Node(
                package='rm_utils',
                executable='lio_extrinsic_receipt.py',
                name='lio_extrinsic_receipt',
                output='screen',
                parameters=[_lio_extrinsic_check_params('fastlio')],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                # ★ 2026-10-21（§4.11）：固定帧跟 level_frame（默认 odom_level=重力对齐的 LIO 世界）。
                #   两档的 .rviz 文件**不改**（`-f` 覆盖它；回滚档给的就是文件里原本的 odom）。
                arguments=['-d', fastlio_rviz_cfg_dir, '-f', _LevelFrame()],
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
                    # ★ 2026-10-21（契约 §4.12）：LIO 外参从 URDF 解析（同 fastlio 那条注释）
                    # ★ 2026-10-27（契约 §4.12.5）：point_lio 也只读 `mapping.extrinsic_T/R`
                    #   （parameters.cpp:179–183）；原来传的顶层键是**死键** ⇒ 运行期
                    #   `mapping.extrinsic_R = I`（measurements §34.3#4），robot11 的 +30° 丢失。
                    {'use_sim_time': use_sim_time, **_lio_extrinsic_params('pointlio')}
                ],
                # T2：统一里程计话题为 /odom（源话题 aft_mapped_to_init）
                remappings=[('/aft_mapped_to_init', '/odom'), ('aft_mapped_to_init', '/odom')],
            ),
            # ★ 2026-10-27（契约 §4.12.5）：运行期断言（读回 `/laserMapping` 生效的
            #   `mapping.extrinsic_T/R`，与 URDF 解析值比对；不一致/死键 ⇒ ERROR + exit 1）。
            Node(
                package='rm_utils',
                executable='lio_extrinsic_receipt.py',
                name='lio_extrinsic_receipt',
                output='screen',
                parameters=[_lio_extrinsic_check_params('pointlio')],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                # ★ 2026-10-21（§4.11）：固定帧跟 level_frame（默认 odom_level=重力对齐的 LIO 世界）。
                #   两档的 .rviz 文件**不改**（`-f` 覆盖它；回滚档给的就是文件里原本的 odom）。
                arguments=['-d', pointlio_rviz_cfg_dir, '-f', _LevelFrame()],
                condition = IfCondition(use_lio_rviz),
            )
        ]),

        # ===== lio:=small_point_lio（vendored Yancey2023/small_point_lio，MIT，pin 688d75c）=====
        # 与上面两个槽位的**本质区别**：它自己就发 `odom→base_link`（源码硬编码），所以**默认档**
        # （有效值 off —— 含默认 `auto` + 默认 `lio:=fastlio`；见契约 §4.10）下：
        #   ① /Odometry → /odom 仍要 remap（统一话题口径）；
        #   ② **绝不能**再起 lio_tf_adapter（否则 odom→base_link 两个发布者 → 双父边/抖动）；
        #   ③ 它不认 camera_init/body ⇒ T1 回退静态桥也要排除它（见下面 icp_frame_bridge_condition）。
        # 它**需要** base_link→livox_frame 这条静态 TF（源码 lookupTransform(lidar_frame,"base_link")，
        # 失败就丢帧、连 /Odometry 都不发）—— 仿真里由 robot_state_publisher 从 URDF 提供
        # （实测 /tf_static 里有 base_link→livox_frame (0.12,0,0.175)，见 docs/lio_slots.md §2）。
        #
        # ★★ 2026-10-20（契约 v3 §13 / 选项 U）：**有效值 != off 时换成下面第二个 Node** ——
        #   同一个可执行文件/节点名，只是参数不同，两个 definition 互斥（照抄本文件
        #   linefit / p2l 在 robot11_mount:=sensor 上的"两份互斥 Node"写法）：
        #     publish_tf:=false      ⇒ 它不再发 odom→base_link（那是 adapter 的活，契约 I2）
        #     odom_convention:=sensor ⇒ /Odometry 发**原始状态 T_ol**（不再是合成 T_ob）
        #     gravity_align_mode / gravity_align_rpy ⇒ E1 初始化对齐（rpy 由 URDF 自动算）
        #   ⇒ 配合下面新加的 `lio_tf_adapter_spl_node`，`odom→base_link` 全栈**恰好一个**发布者。
        #   ⚠️ 为什么不做成"一个 Node + 条件参数字典"：launch 的 parameters 是静态结构，
        #   给默认路径的节点参数表里多一个键都会让"默认路径逐字节不变"不再可证
        #   （见 tools/scripts/gzsim/launch_increment_diff.py 的 --strip-conditions 口径）。
        #
        # ★★ 2026-10-21（契约 §4.14）：**两份互斥的 Node**（同一个可执行文件/节点名）：
        #   ① **E1 关档**（gravity:=tf|base|raw）= 统一输出契约**减去 E1 参数**：
        #      publish_tf:=false + odom_convention:=sensor ⇒ `odom→base_link` 归 adapter
        #      （父帧 = odom_level（tf）/ odom（base|raw）），本节点一个 gravity_align_* 键都不多；
        #   ② **E1 档**（gravity:=lio）= ① + E1/E1b 参数（publish_tf/sensor 同左）。
        #   ⇒ "E1 在 tf/base/raw 上不存在"是**可证**的：①的参数表里根本没有那 3 个键。
        #   ★ 旧的三份（rollback=off+odom、默认=off+odom_level、E1 臂）已收口：raw/base 不再让
        #     LIO 自己发 `body` 档 TF（那会乘掉 §4.12 的 extrinsic_R，把 30° 倾角吞掉）。
        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'small_point_lio'),
            actions=[
            # 把**实际加载的参数文件**打进日志（惰性解析 ⇒ 正好在这里才求值）：
            # 用户不必去猜"launch 到底给我读了 mid360_sim.yaml 还是 tuned 版"，日志里直接有。
            LogInfo(msg=['lio:=small_point_lio 使用的参数文件 = ', small_point_lio_params]),
            Node(
                # ② **E1 关档**（gravity:=tf|base|raw）：off + 任意 level_frame。E1 不参与，只改**输出**：
                #   publish_tf:=false      ⇒ 本节点不发 odom→base_link（唯一发布者 = adapter）
                #   odom_convention:=sensor⇒ /Odometry = LIO 原始状态 T_ol（表达在 `odom` = 原始世界）
                #   gravity_align_h        ⇒ **只作诊断**（初始化地面 vs URDF h ⇒ 响亮 WARN）
                #   init_settle_hold / init_map_extrinsic ⇒ 硬编码（不等落定 + 初始化地图含 t_IL）
                # ⇒ 配合 `lio_tf_adapter_spl_node`（父帧 = _LevelFrame()：tf ⇒ odom_level，
                #   base|raw ⇒ odom）+（tf 时）`odom_level_tf` 静态边。
                #   ★ 为什么 base/raw 也必须走 adapter：§4.12 之后点云在水平的 livox_frame、
                #   倾角在 extrinsic_R；LIO 自己的 `body` 档 TF 不乘 extrinsic_R ⇒ 它会发布
                #   一个**水平**的 base_link（= 把 30° 倾角吞掉，raw 就不"raw"了，§25 的
                #   43.7% 半盲也复现不出来）。adapter 用同一份 URDF 外参合成 ⇒ base_link
                #   在 odom 里真的斜 30°（raw）或由 odom_level 摆正（tf）。
                #   **没有任何 gravity_align_* 键**（E1 关）。
                condition = IfCondition(PythonExpression([
                    "'", _GravityAlignEffectiveMode(), "' == 'off'"])),
                package='small_point_lio',
                executable='small_point_lio_node',
                name='small_point_lio',
                output='screen',
                parameters=[
                    small_point_lio_params,
                    {'use_sim_time': use_sim_time,
                     'publish_tf': False,
                     'odom_convention': 'sensor',
                     # ★ 2026-10-21（契约 §4.12）：外参从 URDF 解析（点云水平、IMU 斜装）
                     # ★ 2026-10-27（§4.12.5）：SPL 读的是**顶层** `extrinsic_T/R`
                     #   （small_point_lio/parameters.cpp:110–116）⇒ 键名仍走同一张表。
                     **_lio_extrinsic_params('small_point_lio'),
                     # ★★ 2026-10-21（契约 §4.14）：初始化行为**硬编码**（删除了两个 launch 参数）：
                     #   · 不等落定（出生即落定；§32.8 证明门本来就不推迟任何一帧）；
                     #   · 初始化地图一律 `R_IL·p + t_IL`（§30 的根因之一；R_IL=I 时等价于 p+t_IL）；
                     #   · gravity_align_h 只作诊断（第一帧地面 vs URDF h >3 cm ⇒ 响亮 WARN）。
                     'init_settle_hold': 0.0,
                     'init_map_extrinsic': True,
                     'gravity_align_h': [_GravityAlignH()]},
                ],
                remappings=[('/Odometry', '/odom')],
            ),
            Node(
                # 统一输出契约档（**有效值** ∈ urdf|imu|hybrid；默认 auto + small_point_lio
                # ⇒ hybrid，也走这一份）。
                # 参数含义见 src/rm_localization/small_point_lio/src/small_point_lio_node.cpp 的
                # "统一输出契约"注释块；两个发布参数默认值 = 现状，所以只有这一档显式传。
                condition = IfCondition(PythonExpression([
                    "'", _GravityAlignEffectiveMode(), "' != 'off'"])),
                package='small_point_lio',
                executable='small_point_lio_node',
                name='small_point_lio',
                output='screen',
                parameters=[
                    small_point_lio_params,
                    {'use_sim_time': use_sim_time,
                     # ★ 2026-10-21（契约 §4.12）：外参从 URDF 解析（点云水平、IMU 斜装）
                     # ★ 2026-10-27（§4.12.5）：顶层键（与上面那份同一张表）。
                     **_lio_extrinsic_params('small_point_lio'),
                     # ① 本节点不再发 odom→base_link（唯一发布者 = lio_tf_adapter，契约 I2）
                     'publish_tf': False,
                     # ② /Odometry 改发 LIO 原始状态 T_ol（frame_id=odom、child_frame_id=livox_frame）
                     'odom_convention': 'sensor',
                     # ③ E1：初始化时把 URDF 的安装倾角吸收进 odom
                     # ★ 2026-10-20：传**有效档**（auto 已解析；节点永远看不到 auto）
                     'gravity_align_mode': _GravityAlignEffectiveMode(),
                     'gravity_align_rpy': [_GravityAlignRpy()],
                     # ④ E1b：z 原点（契约 v3 §4.3）。`ground` ⇒ odom 的 z=0 = 初始化时刻的地面，
                     #   h 由 _gravity_align_h() 从 URDF 算（不手填）；`sensor` = 旧行为（回退用）。
                     #   ★ 2026-10-21（§4.14）：z 原点固定 `ground`（旧参数已删除）。
                     'gravity_align_z_origin': 'ground',
                     'gravity_align_h': [_GravityAlignH()],
                     # 初始化行为同样硬编码（不等落定 + 初始化地图含 t_IL）
                     'init_settle_hold': 0.0,
                     'init_map_extrinsic': True},
                ],
                remappings=[('/Odometry', '/odom')],
            ),
            # ★ 2026-10-27（契约 §4.12.5）：运行期断言（读回 `/small_point_lio` 生效的
            #   **顶层** `extrinsic_T/R`；两份互斥 Node 同名 ⇒ 这里只会看到一个）。
            Node(
                package='rm_utils',
                executable='lio_extrinsic_receipt.py',
                name='lio_extrinsic_receipt',
                output='screen',
                parameters=[_lio_extrinsic_check_params('small_point_lio')],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                # ★ 2026-10-21（§4.11）：固定帧跟 level_frame（默认 odom_level=重力对齐的 LIO 世界）。
                #   两档的 .rviz 文件**不改**（`-f` 覆盖它；回滚档给的就是文件里原本的 odom）。
                arguments=['-d', small_point_lio_rviz_cfg_dir, '-f', _LevelFrame()],
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
                    {'use_sim_time': use_sim_time,
                     # ★ 2026-10-21（§4.11）：父帧 = level_frame（默认 odom_level；回滚档 = odom，
                     #   值与该 YAML 的默认值相同 ⇒ 运行期参数 dump 逐字节不变）。
                     #   输入帧 = 契约的原始世界 `odom`（节点内置常量；父帧 == 输入帧时走直通分支、
                     #   连 tf2 监听都不建 ⇒ 回滚档 graph/数值不变）。
                     'odom_frame': _LevelFrame()}]
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
                     'rpy': [0.0, 0.0, 0.0],
                     # ★ 2026-10-21（§4.11）：父帧/输入帧（见默认模型那条的同名注释）
                     'odom_frame': _LevelFrame()}]
    )

    # ★ 2026-10-07：robot:=robot11 的 lio_tf_adapter —— 同包/同可执行文件/同节点名，
    #   唯一区别是杆臂（xyz）**和姿态补偿（rpy）**。与前两条互斥 ⇒ 同一时刻只有一个
    #    odom→base_link 发布者（契约不变）。
    #   为什么这次连 rpy 都要给：robot11 的雷达是**斜 30°** 装的，而 imu_link 按
    #   【MID-360 的 IMU 与雷达同壳】这条物理事实跟着雷达一起斜
    #   （好处：三份 LIO 配置的 extrinsic_T=[0,0,0.05]/extrinsic_R=I **一个字节都不用改**）。
    #   ⇒ LIO 的 body 系 = 斜 30° 的 IMU 系 ⇒ 必须用 rpy 把它转回水平的 base_link。
    #   数值由 _LioAdapterRobot11 从 URDF 几何**算**出来（不是手抄），随 URDF/安装方式变；
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
                     'rpy': [_LioAdapterRobot11('rpy')],   # 该 30° 旋转的逆 ⇒ 斜的 body 系转回水平
                     # ★ 2026-10-21（§4.11）：父帧/输入帧（见默认模型那条的同名注释）
                     'odom_frame': _LevelFrame()}]
    )

    # ★★ 2026-10-20（契约 v3 §13 / 选项 U）：`lio:=small_point_lio` **也**起 lio_tf_adapter。
    # ★★ 2026-10-21（契约 §4.14）：**所有 gravity 档都起它** —— SPL 节点统一走
    #    `publish_tf:=false + odom_convention:=sensor`（只发 T_ol），`odom→base_link` 的唯一
    #    发布者 = 本 adapter；父帧由 `_LevelFrame()` 给（tf ⇒ odom_level，base|lio|raw ⇒ odom）。
    #    为什么 raw/base 不能留给 LIO 自己发：§4.12 之后 LIO 的 `body` 档 TF 不含 extrinsic_R，
    #    会把 30° 安装倾角吞掉（raw 就不是 raw、base 的"斜 odom 里 2D scan 无逐点高度"也不成立）。
    #
    # ==================== 三方 `T_bl` 记账（写死在这里，改之前先读）====================
    # 记法（与 docs/frame_contract_v3.md §2.2 一致）：
    #   `T_bl := lookupTransform(livox_frame, "base_link")` = **base_link 在 livox 系里的位姿**
    #           （= URDF 里 `body_to_livox` 关节的位姿，robot11 sensor 档 = Rx(−30°) + t）。
    # 这 30° 倾角在整条链上**只允许被乘三次、每次方向固定**，且**三个数都从 URDF 来**：
    #   ① LIO **内部**：`T_il = (R=I, t=(0,0,0.05))`（livox→IMU body，MID-360 同壳）——
    #      在出点公式里乘**一次**（small_point_lio.cpp 的 `Lidar_R_wrt_IMU·p + Lidar_T_wrt_IMU`）。
    #      ★ 这一处**不含** 30°（倾角在 `base_link→livox_frame` 这条**外部**边上，不在雷达↔IMU 之间）；
    #      它的 0.05 m 又不是"倾斜外参"，所以别把它和下面两处混为一谈。
    #   ② E1 初始化（`gravity_align_rpy` = `_GravityAlignRpy()`）：把 `R_bl⁻¹ = R(base_link←livox_frame)`
    #      （= URDF 关节 rpy，robot11 sensor 档 **−30°**）当成"车体水平时雷达在世界里的姿态"，
    #      写成初始 `kf.x.rotation`，同时把初始地图按出点公式搬进世界系 —— **一次**。
    #      ⇒ odom 从此重力对齐；`base_link` 的倾角**不再**出现在 odom 里。
    #   ③ 本适配器（`xyz`/`rpy` = `_LioAdapterExtrinsic()`）：右乘 `T_bl`（robot11 sensor 档
    #      **rpy = +30°**、`xyz = −Rᵀ·p_imu`）把 LIO 的 body 位姿变成 base_link 位姿 —— **一次**：
    #          q_out = q_lio · q_offset、t_out = t_lio + q_lio·t_offset      （= T_ob = T_ol·T_bl）
    #      ⚠️ 与 ② 的方向**相反**（一个是 R_bl⁻¹ 进状态，一个是 T_bl 进输出）—— 这不是重复抵消：
    #         ② 决定"世界系是哪一系"（odom 竖直），③ 决定"车体在 odom 里的姿态"（base_link 水平）。
    #         把 ③ 的 rpy 写成 −30° 的**同号**、或把它归零，就会得到 30° 的假滚转/假俯仰。
    #  ⇒ 判据（可复核）：`T_ob·T_bl⁻¹ ≈ T_ol`（位姿一致性），且 `rpy(odom→base_link) ≈ 0`（静止）。
    # =============================================================================
    lio_tf_adapter_spl_node = Node(
        # ★ 2026-10-21（§4.11）：这个 adapter 现在**两档都要起** ——
        #   ① E1 实验臂（有效值 != off）：LIO 不发 TF、/Odometry = T_ol ⇒ 由它合成 T_bl；
        #   ② 默认档（有效值 off + level_frame=odom_level）：同上（LIO 的 off 档 Node C 也不发 TF），
        #      且它的**父帧 = odom_level**（左乘静态边）。
        #   只有回滚档（off + level_frame=odom）不起它：那时 odom→base_link 由 LIO 自己发。
        condition = IfCondition(PythonExpression([
            "'", LaunchConfiguration('lio'), "' == 'small_point_lio'"])),
        package='lio_tf_adapter',
        executable='lio_tf_adapter_node',
        name='lio_tf_adapter',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('lio_tf_adapter'), 'config', 'lio_tf_adapter.yaml'),
                    {'use_sim_time': use_sim_time,
                     # 话题：LIO 的 /Odometry 已被 launch remap 成 /odom（统一口径）；
                     # 内容 = T_ol（odom_convention:=sensor 档），所以这里必须**合成**一次 T_bl。
                     'odom_topic': '/odom',
                     'xyz': [_LioAdapterExtrinsic('xyz')],   # T_imu_link←base_link（URDF 几何算，含 0.05 m）
                     'rpy': [_LioAdapterExtrinsic('rpy')],   # = R_bl 的逆（robot11 sensor 档 +30°）
                     # ★ 2026-10-21（§4.11）：父帧 = level_frame（默认 odom_level）+ 输入帧 = odom。
                     #   父帧 != 输入帧 ⇒ 本节点用 TF **左乘**那条静态边 T_odom_level←odom
                     #   （唯一来源 = odom_level_tf；查不到就不发 TF，绝不发错的边）。
                     'odom_frame': _LevelFrame()}]
    )

    # ★★ 2026-10-21（契约 v3 §4.11）：**静态边 `odom → odom_level` 的唯一发布者**。
    #   它把"重力对齐 + 地面 z 原点"从**估计器初始化补丁**（E1）搬到**帧图**上：
    #     · 一条 latched 静态变换（`/tf_static`，TRANSIENT_LOCAL，只发一次）⇒ 连续、不跳变；
    #     · 值只由 **URDF**（`gravity_align_rpy` = 解析出的 base_link←livox 安装 rpy）
    #       + **几何**（`gravity_align_h` = 传感器离地高度，`extrinsic_T` = livox 在 IMU 系）算出
    #       —— 与 E1 同一套数学（"把 body 系量到的重力方向转到 −z" + "把原点放到地面"），
    #       但**只算一次、只发一次**，不再改估计器的状态；
    #     · `input_world` / `input_z` / `output_z` 由 `_level_tf_geometry()` 按 LIO 槽位给：
    #       SPL-off → tilted/sensor（要转 −30° 并把地面放到 0）；pointlio → level/sensor（只平移，
    #       它的 Set_init 已经把世界摆正，**绝不能再转一次**）；E1 实验臂 → level/ground ⇒ 恒等；
    #       none/cartographer → 恒等别名（不接管这两个槽位）。
    #   诊断（只读、只写日志）：`gravity_align_h_check`（拟合地面 ⇒ h_meas，与 E1b 同阈值）
    #   和 URDF↔IMU 的 Δ 互检（2°/5° ⇒ INFO/WARN/ERROR，与 E1 hybrid 同一组阈值；
    #   但**不会**因此改用 IMU 值 —— 静态边的值是标定事实，URDF 是唯一所有者）。
    odom_level_tf_node = Node(
        condition = IfCondition(PythonExpression([
            "'", _LevelFrame(), "' == 'odom_level'"])),
        package='rm_nav_bringup',
        executable='odom_level_tf.py',
        name='odom_level_tf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': 'odom',
            'child_frame': 'odom_level',
            # 安装 rpy 与离地高度：**同一个 URDF 解析器**（与 robot_description 同源，禁手填）
            'gravity_rpy_deg': [_GravityAlignRpy()],
            'sensor_height': [_GravityAlignH()],
            # ★ 2026-10-21（契约 §4.12）：t_IL 从 URDF 解析（robot11 = [0,0,0.05]，与三份 LIO 配置同值）
            'extrinsic_t': [_MountExtrinsic('xyz')],
            'input_world': _LevelTfParam('input_world'),
            'input_z': _LevelTfParam('input_z'),
            'output_z': _LevelTfParam('output_z'),
            # 诊断（只读；`check_*=False` 只关日志，不改发布值）
            'check_ground': True,
            'cloud_topic': '/cloud_registered',
            # ★★ 2026-10-21（Part 1）：z 原点来源 = **`urdf`（默认，确定性）**。
            #   为什么能回到 urdf：`odom` 的 z 原点原本被"出生下沉期间的初始化"抬高 ~0.13 m
            #   （根因 = 初始化地图攒了整段下沉的点云 + 少 t_IL），那件事已经在估计器侧修掉
            #   （`init_settle_hold` / `init_map_extrinsic`，见 docs §30）⇒ 静态边可以直接用
            #   URDF 的 h_imu（= 0.216226 m），地面 z 应落在 0 ± 1 cm，**跑间可比较**。
            #   `measured` 保留：第一帧点云拟合地面（§29 的临时挡板；跑间抖 + h_meas 必然 ERROR）。
            # ★ 2026-10-21（§4.14）：z 原点来源固定 = URDF（确定性；旧参数已删除）
            'z_origin_source': 'urdf',
            'ground_check_delay_s': 0.5,
            'ground_check_timeout_s': 90.0,
            'check_imu': True,
            'imu_topic': '/imu/data',
            'imu_samples': 200,
        }],
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
        guard = _load_map_asset_guard()
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
        # ★ 2026-10-29（meas §46.3）：**per-world mapper 槽** + 两个显式覆盖（<=0 = 用槽）。
        #   2026 不在 _MAPPER_PARAMS_BY_WORLD 里且两个覆盖都是 -1（默认）⇒ mapper_overrides 为空
        #   ⇒ params 与 node 的构造**逐字节不变**（默认路径不变）。
        mapper_overrides = dict(_MAPPER_PARAMS_BY_WORLD.get(world_v, {}))
        mt_dist_raw = context.perform_substitution(
            LaunchConfiguration('slam_min_travel_distance')).strip()
        mt_head_raw = context.perform_substitution(
            LaunchConfiguration('slam_min_travel_heading')).strip()
        try:
            mt_dist = float(mt_dist_raw) if mt_dist_raw else -1.0
            mt_head = float(mt_head_raw) if mt_head_raw else -1.0
        except ValueError:
            raise RuntimeError(
                '[mapper 槽] slam_min_travel_distance/heading 不是数字：%r / %r'
                % (mt_dist_raw, mt_head_raw))
        if mt_dist > 0.0:
            mapper_overrides['minimum_travel_distance'] = mt_dist
        if mt_head > 0.0:
            mapper_overrides['minimum_travel_heading'] = mt_head
        if mapper_overrides:
            params.update(mapper_overrides)
            banner += (
                '\n  · [mapper 槽] world=%s ⇒ %s'
                '（per-world 槽：2027 = meas §48 的 A/B 赢家；2026 保持参数文件原值。'
                'A/B：slam_min_travel_distance:=0.6 slam_min_travel_heading:=0.5；'
                '见 meas §45.2c / §46.3 / §48）'
                % (world_v,
                   ', '.join('%s=%s' % (k, mapper_overrides[k])
                             for k in sorted(mapper_overrides))))
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
                package='rm_utils',
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
                    package='rm_utils',
                    executable='map_resume_check.py',        # scripts= 装到 lib/rm_utils/
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
        guard = _load_map_asset_guard()
        world_v = context.perform_substitution(LaunchConfiguration('world')).strip()
        name_v = context.perform_substitution(LaunchConfiguration('map_name')).strip() or world_v
        autocontinue = _as_bool(context.perform_substitution(LaunchConfiguration('map_autocontinue')),
                                'map_autocontinue')
        allow_mismatch = _as_bool(
            context.perform_substitution(LaunchConfiguration('map_allow_world_mismatch')),
            'map_allow_world_mismatch')
        target_frame_v = (context.perform_substitution(
            LaunchConfiguration('cloud_accumulator_target_frame')) or '').strip() or 'map'
        if target_frame_v not in ('map', 'odom'):
            raise RuntimeError("[launch] cloud_accumulator_target_frame:=%r 不是可用取值（map | odom）"
                               % target_frame_v)
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
                # 累积坐标系（§24.5 鬼影修法）：map（默认）| odom
                'target_frame': target_frame_v,
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

    # ★ 2026-10-21：`band_guard:=True` 才存在的只读诊断节点（默认档 condition 恒假 ⇒ 节点集不变）。
    #   它在 t=0 起（不挡 nav2 的 10 s 时序），自己轮询 /scan、TF 与两张 costmap 的参数服务；
    #   参数**单一真源** = 运行期参数服务（不是 launch 传参）⇒ 不会与深合并/rewrite 后的值漂移。
    band_guard_node = Node(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('band_guard'), "'.lower() == 'true' and (",
            "'", LaunchConfiguration('mode'), "' == 'nav' or '",
            LaunchConfiguration('mode'), "' == 'slam_nav')"])),
        package='rm_utils',
        executable='band_guard.py',
        name='band_guard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',
            'costmap_nodes': ['/local_costmap/local_costmap',
                              '/global_costmap/global_costmap'],
            'period_s': 5.0,
            'params_refresh_s': 30.0,
            'min_scan_rate_hz': 1.0,
            'scan_stale_s': 1.0,
            'inband_warn_fraction': 0.10,
            'y_onesided_margin_m': 0.05,
            'rate_limit_s': 5.0,
            'status_topic': '/band_guard/status',
            'publish_status': True,
        }],
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
            'local_obstacle': _EffectiveLocalObstacle(),
            'nav_rviz': use_nav_rviz}.items()
    )

    # 在mapping模式下且不使用slam时，加载空地图
    # map_server现在由lifecycle_manager管理

    ld = LaunchDescription()

    # ★ 2026-10-27（契约 §4.15）：已删除参数的 **t=0 守卫** —— 必须是**第一个** Action：
    #   传了 `robot11_mount:=…` / `turret_yaw_deg:=…` / `turret_pitch_deg:=…` 就在这里停，
    #   任何节点都不会被起（ROS 2 Humble 对未知 launch 参数本身是静默接受的，见 §28.4）。
    ld.add_action(OpaqueFunction(function=_check_removed_launch_args))

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
    ld.add_action(declare_sim_backend_cmd)
    # ★ 2026-10-27（契约 §4.15）：`turret_yaw_deg` / `turret_pitch_deg` / `robot11_mount` 已删除
    #   （云台固定 0/0；安装方式只剩 xacro 默认的 sensor）⇒ 这里不再有它们的声明。
    # ★★ 2026-10-21（契约 §4.14）：重力/水平帧的**唯一用户开关**。必须最先声明（下游派生都读它）。
    ld.add_action(declare_gravity_cmd)
    # ---- 以下 6 个 = **硬报错别名**（只为让 --show-args 还能看到"它们被 gravity 取代"）----
    ld.add_action(declare_level_frame_cmd)
    ld.add_action(declare_gravity_align_mode_cmd)
    ld.add_action(declare_gravity_align_z_origin_cmd)
    ld.add_action(declare_lio_init_settle_cmd)
    ld.add_action(declare_lio_init_map_extrinsic_cmd)
    ld.add_action(declare_z_origin_source_cmd)
    # ★ Phase 4：robot:=robot11 的视觉 mesh 档位（decimated | full）
    ld.add_action(declare_robot11_visual_cmd)
    # 选了哪个模型，日志里给一句收据（两条互斥；默认那条不改变任何行为）
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot'), "' == ''"])),
        msg='[robot] 模型槽位 = 默认（urdf/sentry_robot_sim_rigid.xacro = 单刚体底盘生成物；'
            '几何与 urdf/sentry_robot_sim.xacro 逐条相等）：雷达 base_link+0.12,0,0.175，'
            'linefit sensor_height=0.226（现行标定，未改）；'
            '⚠️ 与旧默认的差别只有"轮关节 fixed + 轮子对地摩擦 0.02"，见 docs §20'))
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot'), "' == 'hzmirm'"])),
        msg=['[robot] 模型槽位 = hzmirm（urdf/sentry_robot_hzmirm_sim.xacro）：用户给的哨兵 URDF，'
             '雷达在云台头上、离地 ~0.80 m（比默认模型高 0.57 m）⇒ '
             '本次 linefit 用 segmentation_sim_hzmirm.yaml（sensor_height 重新标定 + '
             'gravity_aligned_frame=base_link），lio_tf_adapter 杆臂改为 [0,0,-0.75]。'
             '云台**固定**在 yaw=0 / pitch=0（2026-10-27 起 `turret_yaw_deg`/`turret_pitch_deg` '
             '已删除：云台没有控制器、上游两个关节都是 fixed，它们只是 spawn 时的静态姿态；'
             '0/0 = 与上游 URDF 数值等价）。数值对照/回滚见 docs/robot_models.md']))
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot'), "' == 'robot11'"])),
        msg=['[robot] 模型槽位 = robot11（urdf/sentry_robot_robot11_sim.xacro）：用户的哨兵，'
             '真 mesh 视觉 + 真 inertial；雷达在**底盘**上（不在云台）斜 30° 下俯，离地 0.2595 m；'
             '足印内切 0.300 / 外接 0.3565 m；质量 9.5521 kg。',
             '  · 雷达安装倾角 = 上游 URDF 的 body_to_livox.rpy（生成器烘进 xacro；launch 无冻结常数）',
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
             '  · gravity=', _Gravity(),
             '（tf = odom 保持原始世界、静态边 odom→odom_level 负责重力对齐；',
             'base = 高度语义在 base_link、局部障碍层强制逐点；lio = E1 hybrid 把 odom 摆正；',
             'raw = off 档复现半盲。语义/实测见契约 §4.14 + measurements §33）',
             _MountReadout()]))
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_global_relocalizer_cmd)
    ld.add_action(declare_LIO_cmd)
    ld.add_action(declare_nav_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_mapper_cmd)
    ld.add_action(declare_slam_min_travel_distance_cmd)
    ld.add_action(declare_slam_min_travel_heading_cmd)
    ld.add_action(declare_ground_cmd)
    # ★ 2026-10-27（契约 §4.15）：`_check_robot11_mount_combination` 已随 `robot11_mount` 删除
    #   （它检查的 "sensor ⇔ ground:=linefit" 前提已不存在：sensor 语义现在是唯一且无覆盖文件）。
    ld.add_action(declare_global_obstacle_cmd)
    ld.add_action(declare_local_obstacle_cmd)
    # ★★ 2026-10-21（契约 §4.14）：`gravity` 的组合检查 + t=0 收据。
    #   · 6 个旧参数（硬报错别名）在这里**统一**校验：传非默认值 ⇒ t=0 停，消息指向 gravity；
    #   · gravity:=lio 只对 lio:=small_point_lio（有效档解析时抛）；
    #   · gravity:=base 的局部障碍层强制逐点（显式 scan ⇒ 停；默认/混合档 ⇒ 强制 + WARN）；
    #   · gravity:=tf + localization:='' 的既有告警（§4.11）。
    #   位置：必须在 declare_LIO_cmd / declare_local_obstacle_cmd / 6 个别名声明**之后**
    #   （launch 的 DeclareLaunchArgument 按顺序生效），且在所有节点之前（t=0）。
    ld.add_action(OpaqueFunction(function=_check_gravity_contract))
    # ★ 2026-10-20：局部 2D scan 高度带下界（默认 −0.05；回滚 scan_band_min:=0.0，见 §26）
    ld.add_action(declare_scan_band_min_cmd)
    # ★ 2026-10-21：高度带静默过滤守卫（opt-in；默认 False ⇒ 不起节点，见 §17）
    ld.add_action(declare_band_guard_cmd)
    # ★ 2026-10-06：「续建 + 场地隔离」相关（默认值下现有行为不变，详见各自的 description）
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_map_autocontinue_cmd)
    ld.add_action(declare_map_start_pose_cmd)
    ld.add_action(declare_map_allow_world_mismatch_cmd)
    ld.add_action(declare_cloud_accumulator_cmd)
    ld.add_action(declare_cloud_accumulator_target_frame_cmd)
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
    # ★ 2026-10-28（契约 §4.16）：global_relocalizer 槽（默认 none ⇒ condition 假，节点集不变）。
    #   放在 `start_rm_simulation` **之前**：选 scan_context 但 DB 缺失时，`_WorldScDbFile.perform()`
    #   在起 Gazebo 之前就抛错（fail fast，不留半个仿真）；正常档它在 t=0 起，自己等 TF/点云。
    ld.add_action(sc_relocalizer_node)
    ld.add_action(OpaqueFunction(function=_gazebo_model_path_setup))
    ld.add_action(start_rm_simulation)
    # 只有 sim_backend:=gz 才执行；classic 档它是一个 condition 为假的空 Action。
    ld.add_action(start_gz_simulation)
    ld.add_action(bringup_imu_complementary_filter_node)
    # 地面分割槽位：两个节点都进 LaunchDescription，但各自的 condition 保证**只有一个**真的被启动
    #（⇒ /segmentation/obstacle 与 /segmentation/ground 恒定只有一个发布者）。
    # 先有水平点云（/livox/lidar），再做分割/降采样
    ld.add_action(bringup_livox_cloud_extrinsic_node)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    # ★ 2026-10-21（§4.14）：gravity:=base 的 linefit（gravity_aligned_frame=base_link）；与上面互斥
    ld.add_action(bringup_linefit_ground_segmentation_node_base)
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
    # ★ 2026-10-21（契约 §4.11）：静态边 odom→odom_level 的唯一发布者（默认档才起）。
    #   放在 adapter 之前：静态边先在 /tf_static 上（latched），adapter 第一次 lookup 就能拿到
    #   （adapter 侧也有"查不到就不发 TF + 限速 WARN"的兜底，不依赖这个顺序）。
    ld.add_action(odom_level_tf_node)
    # 两条互斥的适配器（默认模型 / robot:=hzmirm），同一时刻只有一个真的启动
    ld.add_action(lio_tf_adapter_node)
    ld.add_action(lio_tf_adapter_hzmirm_node)
    ld.add_action(lio_tf_adapter_robot11_node)
    # ★ 2026-10-20：lio:=small_point_lio 的适配器（**只在统一输出契约档**启动；与上面三条互斥）
    ld.add_action(lio_tf_adapter_spl_node)
    # ★ 续建/从零的判定与横幅在 **t=0** 就执行（OpaqueFunction 是 Action，`--show-args` 不执行它）；
    #   slam_toolbox 节点本身仍由 OpaqueFunction 内部的 TimerAction 延后 4 s 起 ⇒ 时序与改造前一致。
    ld.add_action(start_mapping)
    ld.add_action(start_cloud_accumulator)
    ld.add_action(TimerAction(period=4.0, actions=[start_cartographer_mapping,
                                                   start_cartographer_as_lio_mapping]))
    ld.add_action(TimerAction(period=10.0, actions=[start_navigation2]))
    # ★ 2026-10-21：band_guard（opt-in；默认 condition 假 ⇒ 该 Action 不执行、节点集不变）
    ld.add_action(band_guard_node)
    # 纯建图模式的 RViz（nav2 未启动时 nav_rviz 仍要能出图）
    ld.add_action(mapping_rviz_node)

    return ld
