from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tafl_wcs'

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
    description='TAFL-based Warehouse Control System for RosAGV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tafl_wcs_node = tafl_wcs.tafl_wcs_node:main',
        ],
    },
)
