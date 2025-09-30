from setuptools import find_packages, setup
from glob import glob

package_name = 'cargo_mover_agv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yazelin@ching-tech.com',
    description='Cargo 型 AGV 實作，專門用於物料搬運',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_core_node = cargo_mover_agv.agv_core_node:main',  # ROS node's entry point
            # ROS node's entry point
            'test_agv_core_node = cargo_mover_agv.test_agv_core_node:main',
            # ROS node's entry point
            'agv_robot = cargo_mover_agv.robot_states.robot:main',
            'test_hokuyo_publisher = cargo_mover_agv.test_hokuyo:main',
        ],
    },
)
