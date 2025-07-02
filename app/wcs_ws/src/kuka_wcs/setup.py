from setuptools import find_packages, setup
import glob

package_name = 'kuka_wcs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akifally',
    maintainer_email='akifally@ching-tech.com',
    description='KUKA WCS (Warehouse Control System) for AGV task management and decision making',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_wcs_node = kuka_wcs.kuka_wcs_node:main',
        ],
    },
)
