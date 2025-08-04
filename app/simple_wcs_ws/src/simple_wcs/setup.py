from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'simple_wcs'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.md')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RosAGV Development Team',
    maintainer_email='dev@rosagv.com',
    description='Simple WCS - 極簡化配置驅動的 WCS 系統',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_wcs_node = simple_wcs.wcs_engine:main',
        ],
    },
)