import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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

    ################################ robot_description parameters start ###############################
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'config', 'simulation', 'measurement_params_sim.yaml')))
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
    # nav2 参数已回归自研 rm_navigation 包 params/（R1）
    nav2_params_file_dir = os.path.join(get_package_share_directory('rm_navigation'), 'params', 'nav2_params_sim.yaml')
    ################################### navigation2 parameters end ####################################

    ################################ icp_registration parameters start ################################
    icp_pcd_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'PCD', world]), ".pcd"
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

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='RMUL2026',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Choose localization method: slam_toolbox, amcl, icp')

    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fastlio',
        description='Choose lio alogrithm: fastlio or pointlio')

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
                    {'runtime_pos_log_enable': False}
                ],
                output='screen',
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
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', pointlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            )
        ])
    ])

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition = LaunchConfigurationEquals('localization', 'slam_toolbox'),
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
                condition = LaunchConfigurationEquals('localization', 'amcl'),
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file_dir}.items()
            ),

            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        condition=LaunchConfigurationEquals('localization', 'icp'),
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
                condition = LaunchConfigurationNotEquals('localization', 'slam_toolbox'),
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
        # spin_speed 已回归 fake_vel_transform 包 config/（R1），此处仅留装配级 use_sim_time
        parameters=[os.path.join(get_package_share_directory('fake_vel_transform'), 'config', 'fake_vel_params.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # 添加TF桥接节点，将LIO的frame映射到标准frame
    tf_bridge_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_bridge_camera_init_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    tf_bridge_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_bridge_body_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 只保留必要的静态TF变换
    static_tf_base_link_to_base_link_fake = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_base_link_fake',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_link_fake'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 在mapping模式下也需要启动map_server来显示静态地图（如果存在）
    # 但只在nav模式下才加载静态地图进行定位
    # 对于mapping模式，我们主要依赖LIO的点云地图
    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': empty_map_dir,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz}.items()
    )

    # 在mapping模式下且不使用slam时，加载空地图
    # map_server现在由lifecycle_manager管理

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_LIO_cmd)

    ld.add_action(start_rm_simulation)
    ld.add_action(bringup_imu_complementary_filter_node)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    ld.add_action(bringup_LIO_group)
    
    # 添加TF桥接节点
    ld.add_action(tf_bridge_node)
    ld.add_action(tf_bridge_node2)
    ld.add_action(static_tf_base_link_to_base_link_fake)
    
    ld.add_action(start_localization_group)
    ld.add_action(bringup_fake_vel_transform_node)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)

    return ld
