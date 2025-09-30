from setuptools import find_packages, setup

package_name = 'web_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'pydantic'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yazelin@ching-tech.com',
    description='基於 FastAPI 的 Web API 閘道，支援 Socket.IO 即時通訊',
    license='Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api_server = web_api.api_server:main',  # ROS node's entry point
        ],
    },
)
