from setuptools import find_packages, setup

package_name = 'joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='USB 搖桿輸入處理，支援 ROS 2 Joy 訊息和 Pygame 元件',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
