from setuptools import setup, find_packages

package_name = 'shared_constants'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaze',
    maintainer_email='yaze.lin.j303@gmail.com',
    description='共享常數定義套件，包含 AGV 和 AGVC 系統共用的常數定義',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)