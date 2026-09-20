from glob import glob

from setuptools import find_packages, setup

package_name = 'fire_resq_world_model'

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
    description='Maintains the robot belief state for victims, fire and rescue status.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_model_node = fire_resq_world_model.world_model_node:main',
        ],
    },
)
