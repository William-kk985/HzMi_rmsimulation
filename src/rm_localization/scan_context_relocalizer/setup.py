import os
from glob import glob

from setuptools import setup

package_name = 'scan_context_relocalizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HzMi',
    maintainer_email='dev@hzmi.local',
    description='Scan Context 全局重定位槽（只发 /initialpose + health，不发 TF）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_context_relocalizer_node = scan_context_relocalizer.node:main',
            'build_sc_db = scan_context_relocalizer.build_db:main',
        ],
    },
)
