from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pratica_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vinicius',
    maintainer_email='vinicius@todo.todo',
    description='Controle de pose e trajetória - Prática 03',
    license='MIT',
    entry_points={
   'console_scripts': [
       'pose_controller = pratica_3.pose_controller:main',
       'pose_controller_3_maneuvers = pratica_3.pose_controller_3_maneuvers:main',
       'mission_controller = pratica_3.mission_controller:main',
   ],
},
)
