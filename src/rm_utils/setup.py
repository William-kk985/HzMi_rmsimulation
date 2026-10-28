import os
from glob import glob

from setuptools import setup

package_name = 'rm_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'README.md']),
        # map_asset_guard.py 是**共享模块**（launch / cloud_accumulator / map_archive.sh
        # 三处共用同一份判定与错误文本），所以除可执行文件外再装一份到 share/scripts/，
        # 让 "按 ROS 包找模块" 的两处 import 有稳定路径。
        (os.path.join('share', package_name, 'scripts'), ['rm_utils/map_asset_guard.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HzMi',
    maintainer_email='dev@hzmi.local',
    description='运行期胶水/诊断节点：地图存档守卫、续建体检、会话播报、LIO 外参收据、高度带守卫',
    license='MIT',
    tests_require=['pytest'],
    # 全部按**文件名**装到 lib/rm_utils/（保留 .py 后缀）：
    #   · launch 里的 Node(package='rm_utils', executable='<name>.py') 才能原样照旧；
    #   · `ros2 run rm_utils <name>.py` 与迁移前的 `ros2 run rm_nav_bringup <name>.py` 一一对应。
    scripts=[
        'rm_utils/lio_extrinsic_receipt.py',
        'rm_utils/map_asset_guard.py',
        'rm_utils/map_resume_check.py',
        'rm_utils/map_session_announcer.py',
        'rm_utils/band_guard.py',
    ],
)
