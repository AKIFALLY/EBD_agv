from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'flow_wcs'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RosAGV Development Team',
    maintainer_email='dev@rosagv.com',
    description='Flow WCS - Linear Flow-based WCS System for AGV Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flow_wcs_node = flow_wcs.flow_wcs_node:main',
            'flow_executor = flow_wcs.flow_executor:main',
            'flow_monitor = flow_wcs.flow_monitor:main',
        ],
    },
)