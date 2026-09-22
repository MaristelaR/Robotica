import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1. Pega o diretório do pacote do seu robô
    model_description = get_package_share_directory('pratica_2')

    # 2. Aponta para arquivo URDF
    robot_urdf_path = os.path.join(
        model_description,
        'models',
        'my_robot',
        'model.urdf'
    )

    # Lê o conteúdo do arquivo URDF como texto para passar para o node de publicação
    with open(robot_urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    # 3. Inclui o launch oficial do Gazebo Classic (gazebo_ros)
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # 4. Nó 'robot_state_publisher': Publica a árvore cinemática do URDF no ROS 2
    # O Gazebo Classic usa isso para ler o URDF convertendo-o internamente.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 5. Nó 'spawn_entity': Pega o robô publicado no tópico e o insere no Gazebo Classic
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',  # Agora puxa do tópico e não de um arquivo isolado
            '-entity', 'my_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    # Retorna todos os componentes para o Launch do ROS 2
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_robot
    ])
