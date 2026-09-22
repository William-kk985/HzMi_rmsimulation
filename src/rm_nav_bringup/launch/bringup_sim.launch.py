import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition

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

    ################################## slam_toolbox parameters start ##################################
    slam_toolbox_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world])
    # slam_toolbox 已源码化，参数回归其自身 config/（R1）
    slam_toolbox_localization_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_localization_sim.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_async_sim.yaml')
    ################################### slam_toolbox parameters end ###################################

    ################################### navigation2 parameters start ##################################
    nav2_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world]), ".yaml"
    empty_map_dir = os.path.join(rm_nav_bringup_dir, 'map', 'empty_map.yaml')
    # nav2 参数已回归自研 rm_navigation 包 params/（R1）；按 nav 选择局部规划器变体
    nav2_params_file_dir = PathJoinSubstitution([
        get_package_share_directory('rm_navigation'), 'params',
        PythonExpression(["'nav2_params_sim_' + '", LaunchConfiguration('nav'), "' + '.yaml'"])
    ])
    # AMCL 初值（map 系，米/弧度）：sim 出生点固定，按 world 自动注入，省掉手动发 /initialpose。
    # 依据（用场地 STL 世界包围盒 vs pgm 已知区域比对得出，见 docs/smoke_test_runbook.md §0.5）：
    #   RMUC / RMUL 的 pgm 是 sim 建图导出 → map 原点 = 机器人出生点 → (0, 0, 0)
    #   RMUL2026 的 pgm 是场地几何生成     → map 系 = world 系     → (4.3, 3.35, 0)
    # ★ 2026-09-23：RMUL2026 的 pgm 已换成 **cartographer 新建的图**（见 docs/mapping/README.md
    #   "本次落盘记录"）。cartographer 的 map 系 = **出生点相对系**（新 yaml origin ≈[-2.2,-3.15]，
    #   与旧世界系图差 ≈(4.3,3.35)）⇒ 新图下出生点在 map 里的坐标就是 (0,0,0)。
    #   ⚠️ 若把旧图放回（map/RMUL2026_world_backup.*），这两个值要改回 4.3 / 3.35。
    amcl_init_x = PythonExpression([
        "{'RMUC': 0.0, 'RMUL': 0.0, 'RMUL2026': 0.0}['",
        LaunchConfiguration('world'), "']"])
    amcl_init_y = PythonExpression([
        "{'RMUC': 0.0, 'RMUL': 0.0, 'RMUL2026': 0.0}['",
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
    icp_registration_params_dir = os.path.join(get_package_share_directory('icp_registration'), 'config', 'icp_registration_sim.yaml')
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

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='仅 mode:=nav 生效。重定位模块: amcl | slam_toolbox（需 .posegraph）| '
                    'icp（需 PCD/<world>.pcd）| cartographer（纯定位，需 map/<world>.pbstream）；'
                    '留空 = 回退用法，直接用 LIO 当绝对定位并由静态桥补帧')

    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fastlio',
        description='里程计源（谁发 odom→base_link）: fastlio | pointlio | '
                    'none（不启动 LIO，需外部提供 odom/TF，如轮式里程计）| '
                    'cartographer（**全包形态**：cartographer 兼任里程计源 → 同时跳过 mapper 槽与 '
                    'localization 槽；mode:=nav 时用 map/<world>.pbstream 做纯定位。'
                    '它不发 /odom 话题 → nav:=teb 不适用，用 rpp/dwb。见 docs/tf_interface_contract.md）')

    declare_nav_cmd = DeclareLaunchArgument(
        'nav',
        default_value='rpp',
        description='Choose local planner variant: rpp | dwb | teb '
                    '(对应 rm_navigation/params/nav2_params_sim_<nav>.yaml)')

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
        description='局部代价地图障碍来源（bench 可切换槽位）: '
                    'scan = 只吃 /scan（默认，原行为；链路单点 + p2l 有 45cm 盲区） | '
                    'cloud = 只吃 /segmentation/obstacle 点云直投（不经 p2l，无盲区） | '
                    'both = 双源冗余（任一路挂掉仍能避障）')

    declare_mapper_cmd = DeclareLaunchArgument(
        'mapper',
        default_value='slam_toolbox',
        description='Choose 2D mapping backend (only mode:=mapping): slam_toolbox | cartographer')

    # Specify the actions
    start_rm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(hzmi_rm_simulation_launch_dir, 'rm_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_description': robot_description,
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
        parameters=[segmentation_params]
    )

    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
        # 参数已回归 pointcloud_to_laserscan 包 config/（R1）
        parameters=[os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'config', 'laserscan_params.yaml')],
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
                    {use_sim_time: use_sim_time},
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
                    'params_file': nav2_params_file_dir,
                    'initial_pose_x': amcl_init_x,
                    'initial_pose_y': amcl_init_y,
                    'initial_pose_z': '0.0',
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
                    )
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                # 仅 nav 模式 + icp（或未选重定位）时才单独起 map_server。
                # ⚠️ 必须带 mode=='nav'：建图模式下 localization 为空，若不判断 mode，
                # map_server 会把【磁盘上的旧 pgm】发到 /map，与 slam_toolbox/cartographer
                # 抢同一个话题，导致 map_saver_cli 可能存下旧图（幽灵墙就是这么留下的）。
                condition = IfCondition(PythonExpression([
                    "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                    LaunchConfiguration('localization'), "' != 'slam_toolbox' and '",
                    LaunchConfiguration('localization'), "' != 'amcl'"])),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    'params_file': nav2_params_file_dir,
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
    lio_tf_adapter_node = Node(
        condition = IfCondition(PythonExpression([
            "'", LaunchConfiguration('lio'), "' != 'none' and '",
            LaunchConfiguration('lio'), "' != 'cartographer'"])),
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
    # amcl / slam_toolbox / icp_registration 三者都会自行发布 map→odom，绝不能再叠加静态桥（否则 map/odom 多父边）。
    icp_frame_bridge_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'nav' and '",
        LaunchConfiguration('localization'), "' == '' and '",
        LaunchConfiguration('lio'), "' != 'none' and '",
        LaunchConfiguration('lio'), "' != 'cartographer'"]))

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

    # 注：base_link→base_link_fake 由 fake_vel_transform 以 20Hz 发布（含云台转角），
    # 原先这里的静态桥是重复发布（P4），已删除。
    
    # 在mapping模式下也需要启动map_server来显示静态地图（如果存在）
    # 但只在nav模式下才加载静态地图进行定位
    # 对于mapping模式，我们主要依赖LIO的点云地图
    # ===== 场景形态（mode）三种，启动集明显不同 =====
    #   mapping  : Gazebo + LIO(+RViz) + 在线 SLAM 后端         —— 无导航栈、无地图加载、无重定位
    #   slam_nav : 上述 + 导航栈（costmap 直接吃在线 SLAM 的 /map 与 map→odom）—— 无 map_server、无重定位
    #   nav      : Gazebo + LIO + 重定位(amcl/slam_toolbox-loc/icp) + 导航栈 —— 无在线 SLAM 后端
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
            'params_file': nav2_params_file_dir,
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
