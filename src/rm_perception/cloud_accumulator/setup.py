import os
from glob import glob

from setuptools import setup

package_name = 'cloud_accumulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HzMi',
    maintainer_email='dev@hzmi.local',
    description='3D 点云累加器：map 系累积 + 按名字存档/续建（含场地隔离守卫）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cloud_accumulator_node = cloud_accumulator.accumulator_node:main',
        ],
    },
)
