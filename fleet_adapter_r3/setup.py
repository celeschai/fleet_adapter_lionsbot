import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'fleet_adapter_r3'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi>=0.79.0',
        'uvicorn>=0.18.2',
        'nudged>=0.3',
        'websocket-client',
    ],
    zip_safe=True,
    maintainer='Celes Chai Jia Xuan',
    maintainer_email='jia.xuan@lionsbot.com',
    description='Fleet adapter for R3 robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_r3.fleet_adapter:main'
        ],
    },
)
