import os
import shutil

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Verifica se o xterm está instalado
    if shutil.which('xterm') is None:
        raise RuntimeError(
            "\n\n"
            "ERRO: xterm não está instalado.\n"
            "Instale com:\n"
            "sudo apt install xterm\n"
        )

    pratica_2_dir = get_package_share_directory('pratica_2')

    # Gazebo + robô + ros2_control + controladores
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pratica_2_dir,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    #Controle pelo teclado
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',

        #Abre o teleop em outro terminal para receber as teclas
        prefix='xterm -e',

        output='screen',

        #diff_drive_controller usa TwistStamped
        parameters=[
            {
                'stamped': True,
                'speed': 0.2,
                'turn': 0.5
            }
        ],

        #Liga cmd_vel do teleop ao tópico do controlador
        remappings=[
            (
                'cmd_vel',
                '/diff_drive_controller/cmd_vel'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        teleop
    ])
