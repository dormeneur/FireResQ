from glob import glob

from setuptools import find_packages, setup

package_name = 'fire_resq_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['index.html']},
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
    description='The bottle-seeking demo for the real robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_node = fire_resq_demo.demo_node:main',
            'fake_robot_node = fire_resq_demo.fake_robot_node:main',
        ],
    },
)
