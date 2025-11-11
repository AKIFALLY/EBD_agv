from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kuka_wcs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 加入 launch 檔案
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch*.py'))),
        # 加入 config 檔案
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yaze.lin.j303@gmail.com',
    description='KUKA WCS - KUKA warehouse control system for rack movement and rotation',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_wcs_node = kuka_wcs.kuka_wcs_node:main',
        ],
    },
)
