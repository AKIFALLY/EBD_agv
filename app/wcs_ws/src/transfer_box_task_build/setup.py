from setuptools import setup, find_packages
import os
from glob import glob


package_name = 'transfer_box_task_build'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安裝 launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RosAGV Team',
    maintainer_email='team@rosagv.com',
    description='Transfer Box Task Build - 通用傳送箱 PLC 雙向通訊與自動任務建立套件',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transfer_box_task_build_node = transfer_box_task_build.transfer_box_task_build_node:main',
        ],
    },
)
