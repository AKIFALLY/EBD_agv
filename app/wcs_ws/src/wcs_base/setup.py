from setuptools import find_packages, setup

package_name = 'wcs_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task_condition_query.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wcs_base_node = wcs_base.wcs_base_node:main',
            'task_condition_query_node = wcs_base.task_condition_query_node:main',
            'task_condition_query_cli = wcs_base.task_condition_query_cli:main',
        ],
    },
)
