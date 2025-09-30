from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rcs'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yaze.lin.j303@gmail.com',
    description='機器人控制系統，負責 AGV 車隊協調與任務派發',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rcs_core = rcs.rcs_core:main',
        ],
    },
)
