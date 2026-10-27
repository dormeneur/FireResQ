from glob import glob

from setuptools import find_packages, setup

package_name = 'fire_resq_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Bharti',
    maintainer_email='adityabharti1214@gmail.com',
    description='Physical hardware drivers. Isolated so nothing above it depends on hardware.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = fire_resq_hardware.camera_node:main',
            'base_node = fire_resq_hardware.base_node:main',
            'image_decoder_node = fire_resq_hardware.image_decoder_node:main',
            'motor_check = fire_resq_hardware.motor_check:main',
            'ir_node = fire_resq_hardware.ir_node:main',
        ],
    },
)
