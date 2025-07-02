import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'agv_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
           ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yaze.lin.j303@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'test_agv_node=agv_base.test_agv_node:main',
            'agv_node_base=agv_base.agv_node_base:main',
            'test_agv_node_event=agv_base.test_agv_node_event:main',
        ],
    },
)
