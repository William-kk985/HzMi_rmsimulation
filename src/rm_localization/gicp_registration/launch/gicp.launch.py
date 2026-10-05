import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('gicp_registration'), 'launch'))

def generate_launch_description():
  from launch_ros.actions import Node
  from launch import LaunchDescription

  # 单独起本节点的最小 launch（不带 LIO / nav2）。
  # 正式用法是 bringup_sim.launch.py 的 localization:=gicp 槽 —— 那里会注入
  # use_sim_time 与 pcd_path=PCD/<world>.pcd（见 docs/localization_slots.md §1）。
  params = os.path.join(
    get_package_share_directory('gicp_registration'), 'config', 'gicp_registration_sim.yaml')
  node = Node(
    package='gicp_registration',
    executable='gicp_registration_node',
    name='gicp_registration',
    output='screen',
    parameters=[params]
  )

  return LaunchDescription([node])
