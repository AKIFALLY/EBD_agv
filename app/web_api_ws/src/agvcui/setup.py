from setuptools import find_namespace_packages, find_packages, setup

package_name = 'agvcui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(exclude=['test']),
    package_data={
        'agvcui': [
            'static/**/*',
            'templates/**/*',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yaze.lin.j303@gmail.com',
    description='AGV 車隊管理 Web 介面，支援即時監控與控制',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agvc_ui_server = agvcui.agvc_ui_server:main',
            'test_task_crud = agvcui.testing.test_task_crud_fix:main',
            'test_task_crud_full = agvcui.testing.test_task_crud:main',
        ],
    },
)
