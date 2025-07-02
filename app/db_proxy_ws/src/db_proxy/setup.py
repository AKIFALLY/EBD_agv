from setuptools import find_packages, setup

package_name = 'db_proxy'

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
    maintainer_email='yaze.lin.j303@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agvc_database_node = db_proxy.agvc_database_node:main',
            'test_query_client = db_proxy.query_client:main',
            'db_install = db_proxy.sql.db_install:main',
            'agvc_database_publish_node = db_proxy.agvc_database_publish_node:main',
            'test_fetch_client_node = db_proxy.test_fetch_client_node:main',
            'carrier_query_client = db_proxy.carrier_query_client:main'
        ],
    },
)
