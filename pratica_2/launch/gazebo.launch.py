import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Diretório do pacote
    model_description = get_package_share_directory('pratica_2')

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

    # Substitui $(find pratica_2) pelo caminho real do pacote
    robot_description_content = robot_description_content.replace(
        '$(find pratica_2)', model_description
    )

    # Gazebo Classic
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

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Spawn do robô no Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.3',
        ],
        output='screen'
    )

    # Carrega joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # Carrega diff_drive_controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # Só carrega os controladores depois que o robô foi spawnado
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        load_controllers
    ])
