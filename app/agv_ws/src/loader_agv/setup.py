from setuptools import find_packages, setup
from glob import glob

package_name = 'loader_agv'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yazelin@ching-tech.com',
    description='Loader 型 AGV 實作，專門用於裝載作業',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    test_suite='test',
    entry_points={
        'console_scripts': [
            'loader_agv_node = loader_agv.agv_core_node:main',  # Loader AGV specific entry point
            'test_loader_agv_node = loader_agv.test_agv_core_node:main',  # Test ROS node's entry point
        ],
    },
)
