from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensorpart'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 新增：包含 launch 檔案
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yaze.lin.j303@gmail.com',
    description='感測器整合套件，支援 3D 相機和 OCR 系統',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_sensorpart_node = sensorpart.test_sensorpart_node:main',
            'sensorpart = sensorpart.sensorpart:main',
            'sensorpart_publisher_node = sensorpart.sensorpart_publisher_node:main',
            'sensorpart_simulator_node = sensorpart.sensorpart_simulator_node:main',
        ],
    },
)
