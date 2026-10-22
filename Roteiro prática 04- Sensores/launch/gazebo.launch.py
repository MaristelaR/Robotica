import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Diretório do pacote
    model_description = get_package_share_directory('pratica_4')

    # Caminho do URDF
    robot_urdf_path = os.path.join(
        model_description,
        'models',
        'my_robot',
        'model.urdf'
    )

    # Lê o URDF
    with open(robot_urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    # Diretório do Gazebo
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Inicia o Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # Publica a descrição e as transformações do robô
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Insere o robô no Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.11'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_robot
    ])
