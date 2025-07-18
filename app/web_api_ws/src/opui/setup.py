from setuptools import find_namespace_packages, setup

package_name = 'opui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(exclude=['test']),
    package_data={
        'opui': [
            'frontend/static/*',
            'frontend/static/**/*',
            'frontend/templates/*',
            'frontend/templates/**/*',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi>=0.68.0',
        'uvicorn>=0.15.0',
        'python-socketio>=5.0.0',
        'sqlmodel>=0.0.6',
        'psycopg2-binary>=2.9.0',
        'jinja2>=3.0.0',
        'python-multipart>=0.0.5',
    ],
    extras_require={
        'test': [
            'pytest>=7.0.0',
            'pytest-asyncio>=0.21.0',
            'pytest-mock>=3.10.0',
            'pytest-cov>=4.0.0',
            'httpx>=0.24.0',
        ],
    },
    zip_safe=True,
    maintainer='root',
    maintainer_email='yaze.lin.j303@gmail.com',
    description='OPUI - Operator User Interface for AGV Control System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'op_ui_server = opui.core.op_ui_server:main',
        ],
    },
)
