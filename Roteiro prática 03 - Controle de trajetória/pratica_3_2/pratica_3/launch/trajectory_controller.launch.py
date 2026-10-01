from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_name = 'pratica_3'

    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'trajectory_controller.yaml'
    )

    return LaunchDescription([

        Node(
            package=package_name,
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            parameters=[config_file]
        ),

    ])
