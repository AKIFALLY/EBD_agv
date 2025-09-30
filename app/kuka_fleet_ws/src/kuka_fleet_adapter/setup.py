from setuptools import find_packages, setup

package_name = 'kuka_fleet_adapter'

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
    maintainer_email='yazelin@ching-tech.com',
    description='KUKA Fleet Manager API 適配器，用於 AGV 車隊整合',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_fleet_adapter = kuka_fleet_adapter.kuka_fleet_adapter:main',
        ],
    },
)
