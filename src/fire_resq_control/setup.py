from glob import glob

from setuptools import find_packages, setup

package_name = 'fire_resq_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Bharti',
    maintainer_email='adityabharti1214@gmail.com',
    description='Actuator abstraction: magnet command/state and velocity plumbing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'magnet_node = fire_resq_control.magnet_node:main',
        ],
    },
)
