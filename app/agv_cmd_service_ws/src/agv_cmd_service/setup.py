from setuptools import find_packages, setup
from glob import glob

package_name = 'agv_cmd_service'

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
    maintainer_email='akifally@gmail.com',
    description='手動 AGV 指令服務介面，用於遠端控制',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_cmd_service_node = agv_cmd_service.agv_cmd_service_node:main',
        ],
    },
)
