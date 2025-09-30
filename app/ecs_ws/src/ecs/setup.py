import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ecs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yazelin@ching-tech.com',
    description='設備控制系統，用於 PLC 資料收集和門控制',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ecs_core = ecs.ecs_core:main',  # ROS node's entry point
            'door_controller_node_mqtt = ecs.door_controller_node_mqtt:main',  # ROS node's entry point
        ],
    },
)
